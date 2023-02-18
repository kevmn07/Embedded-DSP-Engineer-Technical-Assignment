#include "pid.hpp"
#include "gtest/gtest.h"

namespace {
// Constructors
TEST(base_pid, Constructors) {

  float tpv0{0}, tsp0{1}, tco0{3}, ttb0{4};
  float tpv1{0}, tsp1{1}, tco1{3}, ttb1{4};
  float tpv2{0}, tsp2{1}, tco2{3}, ttb2{4};
  float rez0, rez1, rez2, rez3, rez4;
  uint64_t tts0{0}, tts10{10},tmpt;

  base_pid pid0;
  base_pid pid1(&tpv1, &tsp1, &tco1, &ttb1);
  base_pid pid2(&tpv2, &tsp2, &tco2, &ttb2, 4, 3, 2, 1);
  
  // Wrong time slice
  EXPECT_EQ(pid0.set_dtmin_param(tts0), -1);
  
  // PID0 is down, process variables are not set
  EXPECT_EQ(pid0.run_pid(tts10), -1);

  // PID1 wait for right time
  EXPECT_EQ(pid1.run_pid(tts0), 0);
  EXPECT_FLOAT_EQ(tco1, 0);

  // PID2 Gain parameters are: 4, 3, 2
  pid2.get_gain_param(rez0, rez1, rez2);
  EXPECT_FLOAT_EQ(rez0, 4);
  EXPECT_FLOAT_EQ(rez1, 3);
  EXPECT_FLOAT_EQ(rez2, 2);
  
  // PID2 runs in Manual mode, CO == Tieback
  EXPECT_EQ(pid2.run_pid(tts10), 0);
  EXPECT_EQ(tco2, ttb2);
}

// Execition
TEST(base_pid, Execution) {

  float tpv0{1}, tsp0{0}, tco0{0}, ttb0{2};
  float tpv1{0}, tsp1{1}, tco1{0}, ttb1{2};
  float tpv2{1}, tsp2{0}, tco2{0}, ttb2{2};
  float tpv3{0}, tsp3{1}, tco3{0}, ttb3{2};
  uint64_t tstep0{1000}, tstep1{2000};
  bool man_sw{false};

  // pid0 - Pterm, pid1 - Iterm,
  // pid2 - DTerm, pid3 - PID,
  base_pid pidP(&tpv0, &tsp0, &tco0, &ttb0, 1, 0, 0, 0);
  base_pid pidI(&tpv1, &tsp1, &tco1, &ttb1, 0, 1, 0, 0);
  base_pid pidD(&tpv2, &tsp2, &tco2, &ttb2, 0, 0, 1, 0);
  base_pid pidPID(&tpv3, &tsp3, &tco3, &ttb3, 1, 1, 1, 0);
   
  pidP.set_man_param(man_sw);
  pidP.run_pid(tstep0);
  pidI.set_man_param(man_sw);
  pidI.run_pid(tstep0);
  pidD.set_man_param(man_sw);
  pidD.run_pid(tstep0);
  pidPID.set_man_param(man_sw);
  pidPID.run_pid(tstep0);

  // StepUp
  EXPECT_FLOAT_EQ(tco0, -1);
  EXPECT_FLOAT_EQ(tco1, 0.001);
  EXPECT_FLOAT_EQ(tco2, -1000);
  EXPECT_FLOAT_EQ(tco3, 1001.001);

  // StepDown
  tpv0 = -1;
  tsp1 = -1;
  tpv2 = -1;
  tsp3 = -1;

  pidP.run_pid(tstep1);
  pidI.run_pid(tstep1);
  pidD.run_pid(tstep1);
  pidPID.run_pid(tstep1);

  EXPECT_FLOAT_EQ(tco0, 1);
  EXPECT_FLOAT_EQ(tco1, 0.0);
  EXPECT_FLOAT_EQ(tco2, 2000);
  EXPECT_FLOAT_EQ(tco3, -2001);
}
}  // namespace
