#pragma once

#include <cmath>
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#define ALTITUDE 0.8f

mavros_msgs::PositionTarget setpoint_raw;

// 飞控状态。
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;
}

// 里程计与起飞初始位置。
tf::Quaternion quat;
nav_msgs::Odometry local_pos;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;



void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
	local_pos = *msg;
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	if (flag_init_position == false && (local_pos.pose.pose.position.z != 0))
	{
		init_position_x_take_off = local_pos.pose.pose.position.x;
		init_position_y_take_off = local_pos.pose.pose.position.y;
		init_position_z_take_off = local_pos.pose.pose.position.z;
		init_yaw_take_off = yaw;
		flag_init_position = true;
	}
}

// 起飞初始位置为平移原点的位置目标。
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
	if (mission_pos_cruise_flag == false)
	{
		mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
		mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
		mission_pos_cruise_flag = true;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = x + init_position_x_take_off;
	setpoint_raw.position.y = y + init_position_y_take_off;
	setpoint_raw.position.z = z + init_position_z_take_off;
	setpoint_raw.yaw = target_yaw;
	ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f), init_pose(%.2f, %.2f, %.2f, %.2f)", 
        local_pos.pose.pose.position.x ,local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, yaw * 180.0 / M_PI, 
        x, y, z, target_yaw * 180.0 / M_PI, 
        init_position_x_take_off, init_position_y_take_off, init_position_z_take_off, init_yaw_take_off * 180.0 / M_PI);
	if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max && fabs(yaw - target_yaw) < 0.1)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		mission_pos_cruise_flag = false;
		return true;
	}
	return false;
}

// 以首次调用时的当前位置为 XY 平移起点；不进行机体航向旋转。
float current_position_cruise_last_position_x = 0;
float current_position_cruise_last_position_y = 0;
bool current_position_cruise_flag = false;
bool current_position_cruise(float x, float y, float z, float target_yaw, float error_max)
{
	if (current_position_cruise_flag == false)
	{
		current_position_cruise_last_position_x = local_pos.pose.pose.position.x;
		current_position_cruise_last_position_y = local_pos.pose.pose.position.y;
		current_position_cruise_flag = true;
	}
	// float p_xy = 0.8;
	setpoint_raw.type_mask = /*1 + 2 + 4 */+ 8 + 16  + 32 + 64 + 128 + 256 + 512 /*+ 1024 */+ 2048;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = x + current_position_cruise_last_position_x;
	setpoint_raw.position.y = y + current_position_cruise_last_position_y;
	setpoint_raw.position.z = z;
	setpoint_raw.yaw = target_yaw;
    ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f)", local_pos.pose.pose.position.x ,local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, yaw * 180.0 / M_PI, x + current_position_cruise_last_position_x, y + current_position_cruise_last_position_y, z, target_yaw * 180.0 / M_PI);
	if (fabs(local_pos.pose.pose.position.x - current_position_cruise_last_position_x - x) < error_max && fabs(local_pos.pose.pose.position.y - current_position_cruise_last_position_y - y) < error_max && fabs(local_pos.pose.pose.position.z - z) < error_max && fabs(yaw - target_yaw) < 0.1)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		// current_position_cruise_flag = false;
		setpoint_raw.yaw_rate = 0.0; // 重置yaw_rate
		setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 + /*1024 + */2048;
		setpoint_raw.yaw = yaw; // 确保最终yaw角度正确
		return true;
	}
	return false;
}

// 降落。
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
ros::Time precision_land_last_time;
bool precision_land()
{
	if (!precision_land_init_position_flag)
	{
		precision_land_init_position_x = local_pos.pose.pose.position.x;
		precision_land_init_position_y = local_pos.pose.pose.position.y;
        precision_land_last_time = ros::Time::now();
		precision_land_init_position_flag = true;
	}
	setpoint_raw.position.x = precision_land_init_position_x;
	setpoint_raw.position.y = precision_land_init_position_y;
	setpoint_raw.position.z = -0.15;
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
    if(ros::Time::now() - precision_land_last_time > ros::Duration(5.0))
    {
        ROS_INFO("Precision landing complete.");
        precision_land_init_position_flag = false; // Reset for next landing
        return true;
    }
    return false;
}

// 任务计时器。
ros::Time last_time;
bool lib_time_init_flag = true;
bool lib_time_record_func(float duration, ros::Time current_time) {
      // 静态变量，记录上次触发时间
	if(lib_time_init_flag)
	{
		last_time = current_time;
		lib_time_init_flag = false; // 只在第一次调用时初始化
	}

    ros::Duration elapsed = current_time - last_time;

    // 如果已经过去超过 duration 秒，返回 true 并更新时间
    if (elapsed.toSec() >= duration) {
		lib_time_init_flag = true;
		return true;
    }
    return false;
}

// 保持当前位置悬停。
bool hover_started = false;
ros::Time hover_last_time;
float hover_x = 0;
float hover_y = 0;
float hover_z = 0;
float hover_yaw = 0;
bool hover(float time_duration)
{
    if(!hover_started)
    {
        hover_last_time = ros::Time::now();
        hover_started = true;
        hover_x = local_pos.pose.pose.position.x;
        hover_y = local_pos.pose.pose.position.y;
        hover_z = local_pos.pose.pose.position.z;
        hover_yaw = yaw;
    }
    ros::Duration elapsed = ros::Time::now() - hover_last_time;
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = hover_x;
    setpoint_raw.position.y = hover_y;
    setpoint_raw.position.z = hover_z;
    setpoint_raw.yaw = hover_yaw;
    ROS_INFO("Hovering at (%.2f, %.2f, %.2f) for %.2f seconds", hover_x, hover_y, hover_z, elapsed.toSec());
     // 如果已经过去超过 time_duration 秒，返回 true 并重置状态 
    if (elapsed.toSec() >= time_duration) {
        hover_started = false; // Reset for next hover
        return true;
    }
    return false;
}




 // ---------------------任务二：速度控制飞行---------------------- //
 
float vel_target_x = 0;
float vel_target_y = 0;
float vel_target_z = 0;
bool vel_flag = false;

bool mission_velocity_cruise(float x, float y, float z, float target_yaw,
                             float error_max, float k, float v_max)
{
    // ① 第一次调用：把相对目标换算成世界坐标并记住（只做一次）
    if (!vel_flag)
    {
        vel_target_x = x + init_position_x_take_off;
        vel_target_y = y + init_position_y_take_off;
        vel_target_z = z + init_position_z_take_off;
        vel_flag = true;
    }

    // ② 算误差（目标 − 当前）
    float ex = vel_target_x - local_pos.pose.pose.position.x;
    float ey = vel_target_y - local_pos.pose.pose.position.y;
    float ez = vel_target_z - local_pos.pose.pose.position.z;

    // ③ 速度 = 比例系数 × 误差
    float vx = k * ex ;
    float vy = k * ey ;
    float vz = 0.0;              // 题目要求：竖直目标速度为 0

    // ④ 限速：限制“水平合速度”不超过 v_max
    // 限制分速度：当两个轴各 1.0 时，合速度是 1.41，实际超了，故不采取
    // 合速度 = 两个分量的平方和开根号
    
    float v_norm = sqrt(vx*vx + vy*vy);
    if (v_norm > v_max)
    {
	vx = vx / v_norm * v_max;   // 等比缩小，方向不变
	vy = vy / v_norm * v_max;
    }

    // ⑤ 到达判断：各轴误差都小于 error_max，且 yaw 到位
    if (fabs(ex) < error_max && fabs(ey) < error_max &&
        fabs(ez) < error_max && fabs(yaw - target_yaw) < 0.1)
    {
        setpoint_raw.velocity.x = 0;
        setpoint_raw.velocity.y = 0;
        setpoint_raw.velocity.z = 0;
        setpoint_raw.type_mask = 1 + 2 + 4 + 64 + 128 + 256 + 2048;
        setpoint_raw.coordinate_frame = 1;
        setpoint_raw.yaw = 0.0;
        vel_flag = false;        // 复位，方便下次再用
        ROS_INFO("速度控制到达目标点");
        return true;
    }

    // ⑥ 填字段并设掩码
    setpoint_raw.type_mask = 1 + 2 + 4 + 64 + 128 + 256 + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.velocity.x = vx;
    setpoint_raw.velocity.y = vy;
    setpoint_raw.velocity.z = vz;
    setpoint_raw.yaw = 0.0;

    ROS_INFO("vx=%.2f vy=%.2f  误差(%.2f, %.2f, %.2f)", vx, vy, ex, ey, ez);
    return false;
}


  // ---------------------- 绕圈飞行 ---------------------- //
  
  
  
float circle_radius = 1.5f;   
int   circle_points = 12;     
int   circle_k = 1;           

// 圆周上第 k 个航点（k = 1..n），返回是否到达
bool circle_waypoint(int k, float radius, int n, float z, float error_max)
{
    float theta = 2.0f * M_PI * (float)k / (float)n;   // 0 ~ 2π
    float tx = radius * cos(theta);
    float ty = radius * sin(theta);

    ROS_INFO("circle %d/%d  theta=%.0f deg  target=(%.2f, %.2f)",
             k, n, theta * 180.0f / M_PI, tx, ty);

    // 复用已有的位置巡航：yaw 固定 0
    return mission_pos_cruise(tx, ty, z, 0.0, error_max);
}
  
  
  // ----------------------拓展：pid算法---------------------- //
  
float pid_integral_x = 0, pid_integral_y = 0;
float pid_prev_ex = 0, pid_prev_ey = 0;
ros::Time pid_last_time;
bool pid_time_init = false;

bool mission_velocity_cruise_pid(float x, float y, float z, float target_yaw,
                                 float error_max, float kp, float ki, float kd, float v_max)
{
    static float tx = 0, ty = 0, tz = 0;   // 世界系目标点，只记一次
    static bool init = false;
    if (!init)
    {
        tx = x + init_position_x_take_off;
        ty = y + init_position_y_take_off;
        tz = z + init_position_z_take_off;
        init = true;
    }

    // ① 误差
    float ex = tx - local_pos.pose.pose.position.x;
    float ey = ty - local_pos.pose.pose.position.y;
    float ez = tz - local_pos.pose.pose.position.z;

    // ② 时间间隔 dt —— PID 必须有它，否则积分/微分没有物理意义
    ros::Time now = ros::Time::now();
    double dt = 0.05;                       // 默认 50ms（对应 20Hz）
    if (pid_time_init) dt = (now - pid_last_time).toSec();
    pid_last_time = now;
    pid_time_init = true;
    if (dt <= 1e-4 || dt > 0.5) dt = 0.05;  // 防第一轮/卡顿导致数值爆炸

    // ③ 积分项（带限幅，防积分饱和 windup）
    pid_integral_x += ex * dt;
    pid_integral_y += ey * dt;
    const float I_MAX = 1.0;
    if (pid_integral_x >  I_MAX) pid_integral_x =  I_MAX;
    if (pid_integral_x < -I_MAX) pid_integral_x = -I_MAX;
    if (pid_integral_y >  I_MAX) pid_integral_y =  I_MAX;
    if (pid_integral_y < -I_MAX) pid_integral_y = -I_MAX;

    // ④ 微分项（误差的变化率）
    float d_ex = (ex - pid_prev_ex) / dt;
    float d_ey = (ey - pid_prev_ey) / dt;
    pid_prev_ex = ex;
    pid_prev_ey = ey;

    // ⑤ PID 输出
    float vx = kp * ex + ki * pid_integral_x + kd * d_ex;
    float vy = kp * ey + ki * pid_integral_y + kd * d_ey;

    // ⑥ 限合速度
    float v_norm = sqrt(vx * vx + vy * vy);
    if (v_norm > v_max)
    {
        vx = vx / v_norm * v_max;
        vy = vy / v_norm * v_max;
    }

    // ⑦ 到达：清零所有内部状态，避免影响下一次任务
    if (fabs(ex) < error_max && fabs(ey) < error_max &&
        fabs(ez) < error_max && fabs(yaw - target_yaw) < 0.1)
    {
        setpoint_raw.velocity.x = 0;
        setpoint_raw.velocity.y = 0;
        setpoint_raw.velocity.z = 0;
        setpoint_raw.type_mask = 1 + 2 + 4 + 64 + 128 + 256 + 2048;
        setpoint_raw.coordinate_frame = 1;
        setpoint_raw.yaw = 0.0;
        pid_integral_x = pid_integral_y = 0;
        pid_prev_ex = pid_prev_ey = 0;
        pid_time_init = false;
        init = false;
        ROS_INFO("[PID] 到达目标点");
        return true;
    }

    // ⑧ 发布字段
    setpoint_raw.type_mask = 1 + 2 + 4 + 64 + 128 + 256 + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.velocity.x = vx;
    setpoint_raw.velocity.y = vy;
    setpoint_raw.velocity.z = 0.0;
    setpoint_raw.yaw = 0.0;

    ROS_INFO("[PID] vx=%.2f vy=%.2f e=(%.2f,%.2f) I=(%.2f,%.2f) dt=%.3f",
             vx, vy, ex, ey, pid_integral_x, pid_integral_y, dt);
    return false;
}












