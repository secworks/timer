//======================================================================
//
// tb_timer_core.v
// --------------
// Testbench for the timer block cipher core.
// Testvectors from:
// https://github.com/sebastien-riou/timer-c-ref/blob/master/log.txt
//
// Author: Joachim Strombergson
// Copyright (c) 2019, Assured AB
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or
// without modification, are permitted provided that the following
// conditions are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

`default_nettype none

module tb_timer_core();

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter DEBUG     = 0;
  parameter DUMP_WAIT = 0;

  parameter CLK_HALF_PERIOD = 1;
  parameter CLK_PERIOD = 2 * CLK_HALF_PERIOD;


  //----------------------------------------------------------------
  // Register and Wire declarations.
  //----------------------------------------------------------------
  reg [31 : 0] cycle_ctr;
  reg [31 : 0] error_ctr;
  reg [31 : 0] tc_ctr;
  reg          tb_monitor;

  reg           tb_clk;
  reg           tb_reset_n;
  reg           tb_start;
  reg           tb_stop;
  reg  [31 : 0] tb_prescaler;
  reg  [31 : 0] tb_timer;
  wire [31 : 0] tb_curr_prescaler;
  wire [31 : 0] tb_curr_timer;
  wire          tb_ready;


  //----------------------------------------------------------------
  // Device Under Test.
  //----------------------------------------------------------------
  timer_core dut(
		 .clk(tb_clk),
                 .reset_n(tb_reset_n),
                 .prescaler_value(tb_prescaler),
		 .timer_value(tb_timer),
		 .start(tb_start),
		 .stop(tb_stop),
		 .curr_prescaler(tb_curr_prescaler),
		 .curr_timer(tb_curr_timer),
		 .ready(tb_ready)
                );


  //----------------------------------------------------------------
  // clk_gen
  //
  // Always running clock generator process.
  //----------------------------------------------------------------
  always
    begin : clk_gen
      #CLK_HALF_PERIOD;
      tb_clk = !tb_clk;
    end // clk_gen


  //----------------------------------------------------------------
  // sys_monitor()
  //
  // An always running process that creates a cycle counter and
  // conditionally displays information about the DUT.
  //----------------------------------------------------------------
  always
    begin : sys_monitor
      cycle_ctr = cycle_ctr + 1;
      #(CLK_PERIOD);
      if (tb_monitor)
        begin
          dump_dut_state();
        end
    end


  //----------------------------------------------------------------
  // dump_dut_state()
  //
  // Dump the state of the dump when needed.
  //----------------------------------------------------------------
  task dump_dut_state;
    begin
      $display("State of DUT");
      $display("------------");
      $display("Cycle: %08d", cycle_ctr);
      $display("");
      $display("Inputs and outputs:");
      $display("start: 0x%1x, stop: 0x%1x, ready: 0x%1x",
	       dut.start, dut.stop, dut.ready);
      $display("prescaler_value: 0x%08x, timer_value: 0x%08x",
	       dut.prescaler_value, dut.timer_value);
      $display("");
      $display("Internal state:");
      $display("prescaler_reg: 0x%08x, prescaler_new: 0x%08x",
	       dut.prescaler_reg, dut.prescaler_new);
      $display("prescaler_set: 0x%1x, prescaler_dec: 0x%1x",
	       dut.prescaler_set, dut.prescaler_dec);
      $display("");
      $display("timer_reg: 0x%08x, timer_new: 0x%08x",
	       dut.timer_reg, dut.timer_new);
      $display("timer_set: 0x%1x, timer_dec: 0x%1x",
	       dut.timer_set, dut.timer_dec);
      $display("");
      $display("core_ctrl_reg: 0x%02x, core_ctrl_new: 0x%02x, core_ctrl_we: 0x%1x",
	       dut.core_ctrl_reg, dut.core_ctrl_new, dut.core_ctrl_we);
      $display("");
      $display("");
    end
  endtask // dump_dut_state


  //----------------------------------------------------------------
  // reset_dut()
  //
  // Toggle reset to put the DUT into a well known state.
  //----------------------------------------------------------------
  task reset_dut;
    begin
      $display("--- DUT before reset:");
      dump_dut_state();
      $display("--- Toggling reset.");
      tb_reset_n = 0;
      #(2 * CLK_PERIOD);
      tb_reset_n = 1;
      $display("--- DUT after reset:");
      dump_dut_state();
    end
  endtask // reset_dut


  //----------------------------------------------------------------
  // wait_ready()
  //
  // Wait for the ready flag in the dut to be set.
  //
  // Note: It is the callers responsibility to call the function
  // when the dut is actively processing and will in fact at some
  // point set the flag.
  //----------------------------------------------------------------
  task wait_ready;
    begin
      #(2 * CLK_PERIOD);
      while (!tb_ready)
        begin
          #(CLK_PERIOD);
          if (DUMP_WAIT)
            begin
              dump_dut_state();
            end
        end
    end
  endtask // wait_ready


  //----------------------------------------------------------------
  // init_sim()
  //
  // Initialize all counters and testbed functionality as well
  // as setting the DUT inputs to defined values.
  //----------------------------------------------------------------
  task init_sim;
    begin
      cycle_ctr  = 0;
      error_ctr  = 0;
      tc_ctr     = 0;
      tb_monitor = 0;

      tb_clk     = 0;
      tb_reset_n = 1;

      tb_start     = 1'h0;
      tb_stop      = 1'h0;
      tb_prescaler = 32'h0;
      tb_timer     = 32'h0;
    end
  endtask // init_sim


  //----------------------------------------------------------------
  // test()
  // Runs an encipher, decipher test with given key and plaintext
  // The generated ciphertext is verified with the given ciphertet.
  // The generated plaintxt is also verified against the
  // given plaintext.
  //----------------------------------------------------------------
  task test1;
    begin
      tc_ctr = tc_ctr + 1;
      tb_monitor = 1;

      $display("--- test1 started.");
      dump_dut_state();
      tb_prescaler = 32'h6;
      tb_timer     = 32'h9;
      #(CLK_PERIOD);
      tb_start = 1'h1;
      #(CLK_PERIOD);
      tb_start = 1'h0;
      wait_ready();
      #(CLK_PERIOD);
      tb_monitor = 0;
      $display("--- test1 completed.");
      $display("");
    end
  endtask // test1


  //----------------------------------------------------------------
  // timer_core_test
  //
  // Test vectors from:
  //----------------------------------------------------------------
  initial
    begin : timer_core_test
      $display("--- Simulation of TIMER core started.");
      $display("");

      init_sim();
      reset_dut();

      test1();

      $display("");
      $display("--- Simulation of timer core completed.");
      $finish;
    end // timer_core_test
endmodule // tb_timer_core

//======================================================================
// EOF tb_timer_core.v
//======================================================================
