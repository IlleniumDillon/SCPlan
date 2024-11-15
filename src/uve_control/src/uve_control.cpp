#include "uve_control.hpp"
#include "eigen3/Eigen/Eigen"
UveControl::UveControl()
    : Node("uve_control")
{
    declare_parameter("uve_control.mpc.horizon", 10);
    declare_parameter("uve_control.mpc.dt", 0.5);
    declare_parameter("uve_control.mpc.maxWheelSpeed", 0.3);
    declare_parameter("uve_control.mpc.wheelWidth", 0.238);
    declare_parameter<std::vector<double>>("uve_control.mpc.Q", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    declare_parameter<std::vector<double>>("uve_control.mpc.R", {1.0, 0.0, 0.0, 1.0});
    declare_parameter<std::vector<double>>("uve_control.mpc.Qf", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    Eigen::MatrixXd Q(3, 3);
    Eigen::MatrixXd R(2, 2);
    Eigen::MatrixXd Qf(3, 3);
    std::vector<double> Q_vec = get_parameter("uve_control.mpc.Q").as_double_array();
    std::vector<double> R_vec = get_parameter("uve_control.mpc.R").as_double_array();
    std::vector<double> Qf_vec = get_parameter("uve_control.mpc.Qf").as_double_array();
    Q << Q_vec[0], Q_vec[1], Q_vec[2], Q_vec[3], Q_vec[4], Q_vec[5], Q_vec[6], Q_vec[7], Q_vec[8];
    R << R_vec[0], R_vec[1], R_vec[2], R_vec[3];
    Qf << Qf_vec[0], Qf_vec[1], Qf_vec[2], Qf_vec[3], Qf_vec[4], Qf_vec[5], Qf_vec[6], Qf_vec[7], Qf_vec[8];
    wheelWidth = get_parameter("uve_control.mpc.wheelWidth").as_double();
    RCLCPP_INFO(get_logger(), "Q: %f %f %f %f %f %f %f %f %f", Q(0, 0), Q(0, 1), Q(0, 2), Q(1, 0), Q(1, 1), Q(1, 2), Q(2, 0), Q(2, 1), Q(2, 2));
    mpc_ = std::make_shared<MPC>(
        get_parameter("uve_control.mpc.horizon").as_int(),
        get_parameter("uve_control.mpc.dt").as_double(),
        get_parameter("uve_control.mpc.maxWheelSpeed").as_double(),
        wheelWidth,
        Q, R, Qf
    );
    mpc_rate = 1 / mpc_->dT;

    action_server_ = rclcpp_action::create_server<uve_message::action::UvePathTrack>(
        this,
        "uve_path_track",
        std::bind(&UveControl::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&UveControl::handle_cancel, this, std::placeholders::_1),
        std::bind(&UveControl::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "UveControl has been started.");

    pub_arm_ = create_publisher<uvs_message::msg::UvEmbArm>("uvs_emb_arm", 1);
    pub_emag_ = create_publisher<uvs_message::msg::UvEmbEmag>("uvs_emb_emag", 1);
    pub_kinetics_ = create_publisher<uvs_message::msg::UvEmbKinetics>("uvs_emb_kinetics", 1);
    sub_status_ = create_subscription<uve_message::msg::UveAgentStatus>("uve_agent_status", 1, std::bind(&UveControl::status_callback, this, std::placeholders::_1));
    sub_emb_ = create_subscription<uvs_message::msg::UvEmbStatus>("uvs_emb_status", 1, std::bind(&UveControl::emb_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&UveControl::timer_callback, this));
}

UveControl::~UveControl()
{
    if (future_.valid())
    {
        future_.wait();
    }
}

rclcpp_action::GoalResponse UveControl::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const uve_message::action::UvePathTrack::Goal> goal)
{
    (void) uuid;
    (void) goal;
    RCLCPP_INFO(get_logger(), "Received goal request");
    // if (future_.valid())
    // {
    //     return rclcpp_action::GoalResponse::REJECT;
    // }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UveControl::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<uve_message::action::UvePathTrack>> goal_handle)
{
    (void) goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void UveControl::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<uve_message::action::UvePathTrack>> goal_handle)
{
    using namespace std::placeholders;
    std::thread{std::bind(&UveControl::execute, this, _1), goal_handle}.detach();
    RCLCPP_INFO(get_logger(), "handle_accepted");
    // future_ = std::async(std::launch::async, std::bind(&UveControl::execute, this, _1), goal_handle);
}

void UveControl::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<uve_message::action::UvePathTrack>> goal_handle)
{
    rclcpp::Rate loop_rate(mpc_rate);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<uve_message::action::UvePathTrack::Feedback>();
    auto result = std::make_shared<uve_message::action::UvePathTrack::Result>();

    Eigen::VectorXd x_ref, y_ref, theta_ref, v_ref, w_ref;
    Eigen::VectorXd state(5);

    auto tnow = now();

    for (int traceNum = 0; traceNum < goal->plan_result.size(); traceNum++)
    {
        x_ref.resize(goal->plan_result[traceNum].trace.size());
        y_ref.resize(goal->plan_result[traceNum].trace.size());
        theta_ref.resize(goal->plan_result[traceNum].trace.size());
        v_ref.resize(goal->plan_result[traceNum].trace.size());
        w_ref.resize(goal->plan_result[traceNum].trace.size());
        for (int waypoint = 0; waypoint < goal->plan_result[traceNum].trace.size(); waypoint++)
        {
            x_ref(waypoint) = goal->plan_result[traceNum].trace[waypoint].x;
            y_ref(waypoint) = goal->plan_result[traceNum].trace[waypoint].y;
            theta_ref(waypoint) = goal->plan_result[traceNum].trace[waypoint].theta;
            if (theta_ref(waypoint) < 0)
            {
                theta_ref(waypoint) += 2 * M_PI;
            }
            if (theta_ref(waypoint) >= 2*M_PI)
            {
                theta_ref(waypoint) -= 2 * M_PI;
            }
        }
        mpc_->setTrackReference(x_ref, y_ref, theta_ref, v_ref, w_ref);
        while(1)
        {
            RCLCPP_INFO(get_logger(), "execute");
            if (goal_handle->is_canceling())
            {
                auto d = now() - tnow;
                result->duration.sec = d.seconds();
                result->duration.nanosec = d.nanoseconds();
                goal_handle->canceled(result);
                return;
            }
            RCLCPP_INFO(get_logger(), "1");
            state <<    status_.pose.x, 
                        status_.pose.y, 
                        status_.pose.theta ,
                        (emb_.left_wheel_speed+emb_.right_wheel_speed) / 2,
                        (emb_.right_wheel_speed-emb_.left_wheel_speed) / wheelWidth * 2;
            RCLCPP_INFO(get_logger(), "2");
            Eigen::VectorXd control(2);
            RCLCPP_INFO(get_logger(), "3");
            bool done = mpc_->update(state, control);
            RCLCPP_INFO(get_logger(), "4");
            uvs_message::msg::UvEmbKinetics kinetics;
            kinetics.v = done ? 0 : control(0);
            kinetics.w = done ? 0 : control(1);
            pub_kinetics_->publish(kinetics);
            loop_rate.sleep();
            if (done)
            {
                mpc_->abort();
                break;
            }
        }
    }

    auto d = now() - tnow;
    result->duration.sec = d.seconds();
    result->duration.nanosec = d.nanoseconds();
    goal_handle->succeed(result);
}

void UveControl::status_callback(const uve_message::msg::UveAgentStatus::SharedPtr msg)
{
    status_ = *msg;
    if (status_.pose.theta < 0)
    {
        status_.pose.theta += 2 * M_PI;
    }
    if (status_.pose.theta >= 2*M_PI)
    {
        status_.pose.theta -= 2 * M_PI;
    }
}

void UveControl::emb_callback(const uvs_message::msg::UvEmbStatus::SharedPtr msg)
{
    emb_ = *msg;
}

void UveControl::timer_callback()
{
}
