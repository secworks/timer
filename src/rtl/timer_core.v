//======================================================================
//
// timer_core.v
// ------------
// timer core.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2022, Assured AB
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

module timer_core(
                  input wire           clk,
                  input wire           reset_n,

                  input wire [31 : 0]  prescaler_value,
                  input wire [31 : 0]  timer_value,
                  input wire           start,
                  input wire           stop,

                  output wire [31 : 0] curr_prescaler,
                  output wire [31 : 0] curr_timer,

                  output wire          ready
                  );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  localparam CTRL_IDLE      = 2'h0;
  localparam CTRL_PRESCALER = 2'h1;
  localparam CTRL_TIMER     = 2'h2;
  localparam CTRL_DONE      = 2'h3;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg          ready_reg;
  reg          ready_new;
  reg          ready_we;

  reg [31 : 0] prescaler_reg;
  reg [31 : 0] prescaler_new;
  reg          prescaler_we;
  reg          prescaler_set;
  reg          prescaler_dec;

  reg [31 : 0] timer_reg;
  reg [31 : 0] timer_new;
  reg          timer_we;
  reg          timer_set;
  reg          timer_dec;

  reg [1 : 0]  core_ctrl_reg;
  reg [1 : 0]  core_ctrl_new;
  reg          core_ctrl_we;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign curr_prescaler = prescaler_reg;
  assign curr_timer     = timer_reg;
  assign ready  = ready_reg;


  //----------------------------------------------------------------
  // reg_update
  //----------------------------------------------------------------
  always @ (posedge clk)
    begin: reg_update
      if (!reset_n)
        begin
          ready_reg     <= 1'h1;
	  prescaler_reg <= 32'h0;
	  timer_reg     <= 32'h0;
          core_ctrl_reg <= CTRL_IDLE;
        end
      else
        begin
          if (ready_we) begin
            ready_reg <= ready_new;
	  end

	  if (prescaler_we) begin
	    prescaler_reg <= prescaler_new;
	  end

	  if (timer_we) begin
	    timer_reg <= timer_new;
	  end

          if (core_ctrl_we) begin
            core_ctrl_reg <= core_ctrl_new;
          end
	end
    end // reg_update


  //----------------------------------------------------------------
  // prescaler_ctr
  //----------------------------------------------------------------
  always @*
    begin : prescaler_ctr
      prescaler_new = 32'h0;
      prescaler_we  = 1'h0;

      if (prescaler_set) begin
	prescaler_new = prescaler_value;
	prescaler_we  = 1'h1;
      end
      else if (prescaler_dec) begin
	prescaler_new = prescaler_new - 1'h1;
	prescaler_we  = 1'h1;
      end
    end


  //----------------------------------------------------------------
  // timer_ctr
  //----------------------------------------------------------------
  always @*
    begin : timer_ctr
      timer_new = 32'h0;
      timer_we  = 1'h0;

      if (timer_set) begin
	timer_new = timer_value;
	timer_we  = 1'h1;
      end
      else if (timer_dec) begin
	timer_new = timer_new - 1'h1;
	timer_we  = 1'h1;
      end
    end


  //----------------------------------------------------------------
  // Core control FSM.
  //----------------------------------------------------------------
  always @*
    begin : core_ctrl
      ready_new     = 1'h0;
      ready_we      = 1'h0;
      prescaler_set = 1'h0;
      prescaler_dec = 1'h0;
      timer_set     = 1'h0;
      timer_dec     = 1'h0;
      core_ctrl_new = CTRL_IDLE;
      core_ctrl_we  = 1'h0;

      case (core_ctrl_reg)
        CTRL_IDLE: begin
          if (start)
            begin
              ready_new     = 1'h0;
              ready_we      = 1'h1;
	      prescaler_set = 1'h1;
	      timer_set     = 1'h1;
              core_ctrl_new = CTRL_PRESCALER;
              core_ctrl_we  = 1'h1;
            end
        end


	CTRL_PRESCALER: begin
	  if (stop) begin
            core_ctrl_new = CTRL_DONE;
            core_ctrl_we  = 1'h1;
	  end
	  else begin
	    if (prescaler_reg == 0) begin
              core_ctrl_new = CTRL_TIMER;
              core_ctrl_we  = 1'h1;
	    end
	    else begin
	      prescaler_dec = 1'h1;
	    end
	  end
	end


	CTRL_TIMER: begin
	  if (stop) begin
            core_ctrl_new = CTRL_DONE;
            core_ctrl_we  = 1'h1;
	  end
	  else begin
	    if (timer_reg == 0) begin
              core_ctrl_new = CTRL_DONE;
              core_ctrl_we  = 1'h1;
	    end
	    else begin
	      prescaler_set = 1'h1;
	      timer_dec     = 1'h1;
	    end
	  end
	end


        CTRL_DONE: begin
          ready_new     = 1'h1;
          ready_we      = 1'h1;
          core_ctrl_new = CTRL_IDLE;
          core_ctrl_we  = 1'h1;
        end

        default: begin
        end
      endcase // case (core_ctrl_reg)
    end // core_ctrl

endmodule // timer_core

//======================================================================
// EOF timer_core.v
//======================================================================
