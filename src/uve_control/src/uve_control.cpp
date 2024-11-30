#include "uve_control.hpp"
#include "eigen3/Eigen/Eigen"

using namespace std::chrono_literals;

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
    declare_parameter("uve_control.arm.arm_off");
    declare_parameter("uve_control.arm.arm_on");
    declare_parameter("uve_control.arm.base_off");
    declare_parameter("uve_control.arm.base_on");
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
    arm_arm_off = get_parameter("uve_control.arm.arm_off").as_int();
    arm_arm_on = get_parameter("uve_control.arm.arm_on").as_int();
    arm_base_off = get_parameter("uve_control.arm.base_off").as_int();
    arm_base_on = get_parameter("uve_control.arm.base_on").as_int();

    RCLCPP_INFO(get_logger(), "UveControl has been started.");

    pub_arm_ = create_publisher<uvs_message::msg::UvEmbArm>("uvs_emb_arm", 1);
    pub_emag_ = create_publisher<uvs_message::msg::UvEmbEmag>("uvs_emb_emag", 1);
    pub_kinetics_ = create_publisher<uvs_message::msg::UvEmbKinetics>("uvs_emb_kinetics", 1);
    sub_status_ = create_subscription<geometry_msgs::msg::Pose2D>("uve_agent_status", 1, std::bind(&UveControl::status_callback, this, std::placeholders::_1));
    sub_emb_ = create_subscription<uvs_message::msg::UvEmbStatus>("uvs_emb_status", 1, std::bind(&UveControl::emb_callback, this, std::placeholders::_1));

    control_ref_sub = create_subscription<uve_message::msg::UvePlanResult>("uve_control_ref", 1, std::bind(&UveControl::control_ref_callback, this, std::placeholders::_1));
    control_get_pub = create_publisher<std_msgs::msg::Empty>("uve_control_get", 1);
    control_finish_pub = create_publisher<std_msgs::msg::Empty>("uve_control_finish", 1);
    control_abort_sub = create_subscription<std_msgs::msg::Empty>("uve_control_abort", 1, std::bind(&UveControl::control_abort_callback, this, std::placeholders::_1));
}

UveControl::~UveControl()
{
    if (future_.valid())
    {
        future_.wait();
    }
}

/*void UveControl::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<uve_message::action::UvePathTrack>> goal_handle)
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
            // if (theta_ref(waypoint) < 0)
            // {
            //     theta_ref(waypoint) += 2 * M_PI;
            // }
            // if (theta_ref(waypoint) >= 2*M_PI)
            // {
            //     theta_ref(waypoint) -= 2 * M_PI;
            // }
            if (theta_ref(waypoint) <= -M_PI)
            {
                theta_ref(waypoint) += 2 * M_PI;
            }
            if (theta_ref(waypoint) > M_PI)
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
            state <<    status_.x, 
                        status_.y, 
                        status_.theta ,
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
}*/

void UveControl::status_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    status_ = *msg;
    // if (status_.theta < 0)
    // {
    //     status_.theta += 2 * M_PI;
    // }
    // if (status_.theta >= 2*M_PI)
    // {
    //     status_.theta -= 2 * M_PI;
    // }
    if (status_.theta <= -M_PI)
    {
        status_.theta += 2 * M_PI;
    }
    if (status_.theta > M_PI)
    {
        status_.theta -= 2 * M_PI;
    }
}

void UveControl::emb_callback(const uvs_message::msg::UvEmbStatus::SharedPtr msg)
{
    emb_ = *msg;
}

void UveControl::control_ref_callback(const uve_message::msg::UvePlanResult::SharedPtr msg)
{
    path = *msg;    
    abortFlag.store(false);
    if (future_.valid())
    {
        future_.wait();
    }
    future_ = std::async(std::launch::async, &UveControl::controlTask, this);
}

void UveControl::control_abort_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    abortFlag.store(true);
}

void UveControl::controlTask()
{
    uvs_message::msg::UvEmbArm armOutput;
    uvs_message::msg::UvEmbEmag emagOutput;
    uvs_message::msg::UvEmbKinetics vwOutput;
    // rclcpp::Rate loop_rate(mpc_rate);
    Eigen::VectorXd x_ref, y_ref, theta_ref, v_ref, w_ref;
    Eigen::VectorXd state(5);
    Eigen::VectorXd control(2);
    uint8_t PID_state = 0;
    x_ref.resize(path.trace.size());
    y_ref.resize(path.trace.size());
    theta_ref.resize(path.trace.size());
    v_ref.resize(path.trace.size());
    w_ref.resize(path.trace.size());
    for (int waypoint = 0; waypoint < path.trace.size(); waypoint++)
    {
        x_ref(waypoint) = path.trace[waypoint].x;
        y_ref(waypoint) = path.trace[waypoint].y;
        theta_ref(waypoint) = path.trace[waypoint].theta;
        if (theta_ref(waypoint) <= -M_PI)
        {
            theta_ref(waypoint) += 2 * M_PI;
        }
        if (theta_ref(waypoint) > M_PI)
        {
            theta_ref(waypoint) -= 2 * M_PI;
        }
    }
    // 打开电磁铁
    // if (path.interaction > -1)
    // {
    //     armOutput.arm_arm = arm_arm_on;
    //     armOutput.arm_base = arm_base_on;
    //     emagOutput.enable = true;
    //     pub_arm_->publish(armOutput);
    //     pub_emag_->publish(emagOutput);
    // }
    // std::this_thread::sleep_for(1s);
    // 跟踪第一个点
    // mpc_->setTrackReference(x_ref.block(0,0,1,1), y_ref.block(0,0,1,1), theta_ref.block(0,0,1,1), v_ref.block(0,0,1,1), w_ref.block(0,0,1,1));
    while (_taskAlive())
    {
        auto time_start = std::chrono::high_resolution_clock::now();
        state <<    status_.x, 
                        status_.y, 
                        status_.theta ,
                        (emb_.left_wheel_speed+emb_.right_wheel_speed) / 2,
                        (emb_.right_wheel_speed-emb_.left_wheel_speed) / wheelWidth * 2;
        // 如果当前状态与路径点第一个点的误差很小则退出
        double dx = x_ref(0) - state(0);
        double dy = y_ref(0) - state(1);
        double dtheta = theta_ref(0) - state(2);
        if (dtheta < -M_PI)
        {
            dtheta += 2 * M_PI;
        }
        if (dtheta > M_PI)
        {
            dtheta -= 2 * M_PI;
        }
        RCLCPP_INFO(get_logger(), "error: %f,%f,%f", dx,dy,dtheta);
        if (std::sqrt(dx*dx+dy*dy) < 0.05 && dtheta < 0.17)
        {
            vwOutput.v = 0;
            vwOutput.w = 0;
            pub_kinetics_->publish(vwOutput);
            break;
        }
        // mpc
        // mpc_->update(state, control);

        // PID
        // turn to target point
        if (PID_state == 0)
        {
            double target_theta = std::atan2(dy, dx);
            double dtheta_pid = target_theta - state(2);
            if (dtheta_pid < -M_PI)
            {
                dtheta_pid += 2 * M_PI;
            }
            if (dtheta_pid > M_PI)
            {
                dtheta_pid -= 2 * M_PI;
            }
            if (std::abs(dtheta_pid) < 0.17)
            {
                control(0) = 0;
                control(1) = 0;
                PID_state = 1;
            }
            else
            {
                control(0) = 0;
                control(1) = dtheta_pid * 2;
            }
        }
        else if (PID_state == 1)
        {
            double distance = std::sqrt(dx*dx+dy*dy);
            double target_theta = std::atan2(dy, dx);
            double dtheta_pid = target_theta - state(2);
            if (dtheta_pid < -M_PI)
            {
                dtheta_pid += 2 * M_PI;
            }
            if (dtheta_pid > M_PI)
            {
                dtheta_pid -= 2 * M_PI;
            }
            if (distance < 0.05)
            {
                control(0) = 0;
                control(1) = 0;
                PID_state = 2;
            }
            else 
            {
                control(0) = 0.1;
                control(1) = dtheta_pid * 2;
            }
        }
        else if (PID_state == 2)
        {
            if (std::abs(dtheta) < 0.17)
            {
                control(0) = 0;
                control(1) = 0;
                PID_state = 3;
            }
            else
            {
                control(0) = 0;
                control(1) = dtheta * 2;
            }
        }
        else
        {
            break;
        }
        vwOutput.v = control(0);
        vwOutput.w = control(1);
        pub_kinetics_->publish(vwOutput);
        while (1)
        {
            auto time_end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(time_end - time_start);
            if (duration.count() > mpc_->dT * 1e9)
            {
                break;
            }
        }
    }
    control_get_pub->publish(std_msgs::msg::Empty());
    if (path.interaction > -1)
    {
        armOutput.arm_arm = arm_arm_on;
        armOutput.arm_base = arm_base_on;
        emagOutput.enable = true;
        pub_arm_->publish(armOutput);
        pub_emag_->publish(emagOutput);
        vwOutput.v = 0;
        vwOutput.w = 0;
        pub_kinetics_->publish(vwOutput);
        std::this_thread::sleep_for(1s);
    }
    // 跟踪路径
    mpc_->setTrackReference(x_ref, y_ref, theta_ref, v_ref, w_ref);
    while (_taskAlive())
    {
        auto time_start = std::chrono::high_resolution_clock::now();
        state <<    status_.x, 
                        status_.y, 
                        status_.theta ,
                        (emb_.left_wheel_speed+emb_.right_wheel_speed) / 2,
                        (emb_.right_wheel_speed-emb_.left_wheel_speed) / wheelWidth * 2;
        // mpc
        bool done = mpc_->update(state, control);
        vwOutput.v = control(0);
        vwOutput.w = control(1);
        pub_kinetics_->publish(vwOutput);
        while (1)
        {
            auto time_end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(time_end - time_start);
            if (duration.count() > mpc_->dT * 1e9)
            {
                break;
            }
        }
        if (done)
        {
            break;
        }
    }
    control_finish_pub->publish(std_msgs::msg::Empty());
    // 关闭电磁铁
    armOutput.arm_arm = arm_arm_off;
    armOutput.arm_base = arm_base_off;
    emagOutput.enable = false;
    vwOutput.v = 0;
    vwOutput.w = 0;
    pub_arm_->publish(armOutput);
    pub_emag_->publish(emagOutput);
    pub_kinetics_->publish(vwOutput);
}
