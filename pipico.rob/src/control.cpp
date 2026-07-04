#include <Arduino.h>
#include "robot.h"
#include "sec_timer.h"
#include "state_machines.h"
#include "actions.h"
#include "htransf_2d.h"
#include <WiFi.h>



class LED_fsm_t: public state_machine_t
{
  virtual void next_state_rules(void)
  {
    // Rules for the state evolution
     if(state == 0 && tis > 0.1 * rate) {
      set_next_state(1);

    } else if (state == 1 && tis > 0.2 * rate) {
      set_next_state(2);

    } else if(state == 2 && tis > 0.2 * rate) {
      set_next_state(3);

    } else if(state == 3 && tis > 0.5 * rate) {
      set_next_state(0);
    }
  };


  virtual void state_actions_rules(void)
  {
    // Actions in each state
    if (state == 0) {        // LED on
      robot.led = 1;

    } else if (state == 1) { // LED off
      robot.led = 0;

    } else if (state == 2) { // LED on
      robot.led = 1;

    } else if (state == 3) { // LED off
      robot.led = 0;
    }
  };
  public:
  float rate = 2;
};

LED_fsm_t LED_fsm;


static float seconds(void)
{
  return 1e-3 * millis();
}

class main_fsm_t: public state_machine_t
{
  virtual void next_state_rules(void)\
  {
    int in_dist = (dist(action.Pf.x, action.Pf.y, action.Pi.x, action.Pi.y) < action.e_xy_tresh);
    int in_ang = (fabs(dif_angle(action.thetai, action.thetaf)) < action.e_theta_tresh);

    // Rules for the state evolution
    if (state == 300 && actions_count >= 1){
      set_next_state(301);

    } else if (state == as_opt_trajectory && action.set_last_theta){
      set_next_state(as_set_theta);
      action.set_last_theta = 0;
      action.done = 0;
      
    } else if (state == as_opt_trajectory && action.next_step == true){
      if (in_dist) {
        set_next_state(as_set_theta);
        action.done = 0;

      } else if (in_ang) {
        if (action.blocked_node) {
          set_next_state(as_backwards_walk);
          action.done = 0;
          
        } else {
          set_next_state(as_follow_line);
          action.done = 0;
        }
      } else {
        set_next_state(as_set_theta); // first iteration only
        action.done = 0;
      }

    } else if (state == as_follow_line && action.done && action.robotatfactory){
      set_next_state(as_opt_trajectory);

    } else if (state == as_set_theta && action.done && action.robotatfactory){
      set_next_state(as_opt_trajectory);

    } else if (state == as_opt_trajectory && action.done){
      set_next_state(200);

    } else if (state == as_backwards_walk && action.done == true && action.robotatfactory){
      set_next_state(as_opt_trajectory);

    } else if (state == as_set_theta && action.done && !action.robotatfactory){
      if (action.all_done) {
        set_next_state(200);
      } else {
        set_next_state(as_switch_solenoid);
        action.hold_box = !action.hold_box;
      }

    } else if (state == as_switch_solenoid && tis >= 1) {
      set_next_state(as_robot_at_factory);

    } else if (state == as_robot_at_factory) {
      if (action.all_done){
        set_next_state(200);
      } else {
        set_next_state(as_opt_trajectory);
      }
      

    }
  };


  virtual void state_actions_rules(void)
  {
    static int last_state;
    
    if (state != last_state) {
      robot.u1 = 0;
      robot.PID[0].Se = 0;
      robot.PID[0].y_ref = 0;

      robot.u2 = 0;
      robot.PID[1].Se = 0;
      robot.PID[1].y_ref = 0;
    }

 // Actions in each state
    if (state == as_stop) {   // Robot Stoped
      robot.control_mode = cm_kinematics;
      robot.solenoid_u = 0;
      robot.setRobotVW(0, 0);

    } else if (state == as_set_theta) {  // action set_theta
      if (last_state != as_set_theta)
        action.thetai = robot.thetae;
      action.set_theta();

    } else if (state == as_goto_xy) {  // action goto_xy
      action.goto_xy();

    } else if (state == as_follow_line) {  // action follow_line
      action.follow_line();

    } else if (state == as_follow_circle) {  // action follow_circle
      action.follow_circle();

    } else if (state == as_follow_track) {  // action follow_track
      action.follow_track();

    } else if (state == as_follow_wall_left) {  // action follow_wall_left
      action.follow_wall_left();
    
    } else if (state == as_backwards_walk) { // action backwards_walk
      // robot.solenoid_u = 4.0;
      action.backwards_walk();
    
    } else if (state == as_opt_trajectory) {  // action opt_trajectory
      action.opt_trajectory();

    } else if (state == as_robot_at_factory){ // action robot_at_factory
      action.robot_at_factory();

    } else if (state == as_switch_solenoid) {
      action.switch_solenoid();
      
    } else if (state == 100) {  // Another way to stop the robot
      robot.control_mode = cm_kinematics;
      robot.v_req = 0;
      robot.w_req = 0;

    } else if (state == 101) {  // Use v_req and w_req that can be set from remote commands but stop after _timeout_ seconds
      robot.control_mode = cm_kinematics;
      if (robot.timeout != 0 && robot.vw_timer.past(robot.timeout)) {
        robot.v_req = 0;
        robot.w_req = 0;
      }
      robot.setRobotVW(robot.v_req, robot.w_req);

    } else if (state == 102) {  // Option for remote PID control (The remote commands set w1ref and w2ref)
      robot.control_mode = cm_pid;

    } else if (state == 105) {  // Option for simple line follower
      robot.control_mode = cm_kinematics;

      if (robot.IRLine.seing_line) {
        robot.v_req = robot.Vcenter;
        // Go, possibly, slower if a side sensor detects the line
        if (max(robot.IRLine.IR_values[1], robot.IRLine.IR_values[3]) > robot.IRLine.IR_tresh) {
          robot.v_req = min(robot.v_req, robot.Vside);
        }
        // Go, possibly, even slower if an edge sensor detects the line
        if (max(robot.IRLine.IR_values[0], robot.IRLine.IR_values[4]) > robot.IRLine.IR_tresh) {
          robot.v_req = min(robot.v_req, robot.Vedge);
        }

        // W from the IR sensors that are active
        robot.w_req = 0;
        for(int i = 0; i < robot.IRLine.sensor_count; i++) {
          robot.w_req += robot.IR_W_gain[i] * 1e-3 * robot.IRLine.IR_values[i];

        }
      } else if (robot.IRLine.last_black_timer.time() < robot.no_line_interval) {
        // Keep previous v and w
      } else {
        // Stop if more than 'no_line_interval' seconds without seing a line
        robot.v_req = 0;
        robot.w_req = 0;
      }

      robot.setRobotVW(robot.v_req, robot.w_req);

    } else if (state == 114) {  // action follow_track with localization
      robot.control_mode = cm_kinematics;
      LED_fsm.state = 4; // No blinking

      action.follow_track();


    } else if (state == 200) {  // Direct stop (works even if the PID is unstable)
      robot.control_mode = cm_voltage;
      robot.u1 = 0;
      robot.u2 = 0;
      robot.solenoid_u = 0;

    } else if (state == 201) {  // Option for remote tests (The remote commands set u_req and u2_req)
      robot.control_mode = cm_voltage;
      robot.u1 = robot.u1_req;
      robot.u2 = robot.u2_req;

    } else if (state == 203) {
      robot.send_command("err", "state 203");
      robot.send_command("msg", "state 203");

    }

    last_state = state;
  };
};

main_fsm_t main_fsm;


void init_control(robot_t& robot)
{
  robot.pfsm = &main_fsm;
  main_fsm.force_state(200);
  state_machines.register_state_machine(&main_fsm);
  state_machines.register_state_machine(&LED_fsm);
}

void control(robot_t& robot)
{
  // Speed up the blink rate if we have WiFi connected
  if (WiFi.connected()) {
    LED_fsm.rate = 0.6;
  }

  robot.control_mode = cm_kinematics;
  state_machines.step();
}
