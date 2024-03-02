# SPDX-FileCopyrightText: © 2023 Uri Shaked <uri@tinytapeout.com>
# SPDX-License-Identifier: MIT

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

async def send_packet(dut, data):
  # Clock Cycles per Bit
  cycles_per_bit = 8
  # Start bit (ui_in[2] = 0)
  dut.ui_in.value = 0
  await ClockCycles(dut.clk, cycles_per_bit)
  # Send data bits
  for i in range(0,8):
    if (data & 1) == 1:
      dut.ui_in.value = 4
    else:
      dut.ui_in.value = 0
    await ClockCycles(dut.clk, cycles_per_bit)
    data = data >> 1
  # Stop bit
  dut.ui_in.value = 4
  await ClockCycles(dut.clk, cycles_per_bit)

async def read_packet(dut):
  # Clock Cycles per Bit
  cycles_per_bit = 8
  # Wait for start bit
  while (uo_out.value & 4) == 4:
    await ClockCycles(dut.clk, 1)
  # Wait out start bit, align with middle of first data bit
  await ClockCycles(dut.clk, cycles_per_bit + cycles_per_bit/2)
  # Read data
  data = 0
  for i in range(0,8):
    if (uo_out.value & 4) == 4:
      data = data | 1
    data = data << 1
    await ClockCycles(dut.clk, cycles_per_bit)
  # Wait out rest of packet
  await ClockCycles(dut.clk, cycles_per_bit + cycles_per_bit/2)
  return(data)

async def send_program(dut, file_name):
  # Open binary file, read contents, close file
  bin_file = open(file_name, "rb")
  data_list = list(bin_file.read())
  bin_file.close()
  # Send data to dut
  for data in data_list:
    send_packet(dut, data)

async def run_test_program(dut, program_file_name):
  send_program(dut, program_file_name):
  await ClockCycles(dut.clk, 100) # Give a good amount of time for program to terminate
  return(read_data(dut))

async def read_data(dut):
  send_packet(dut, 0x55)
  data_list = list()
  for i in range(0,16):
    data_list.append(read_packet(dut))
  return(data_list)

@cocotb.test()
async def test_top(dut):
  dut._log.info("Start")
  
  # Our example module doesn't use clock and reset, but we show how to use them here anyway.
  clock = Clock(dut.clk, 10, units="us")
  cocotb.start_soon(clock.start())

  # Reset
  dut._log.info("Reset")
  dut.ena.value = 1
  dut.ui_in.value = 4    # Set ui_in[2] to 1, UART inactive value
  dut.uio_in.value = 0
  dut.rst_n.value = 0
  await ClockCycles(dut.clk, 10)
  dut.rst_n.value = 1

  # Run "sum_one_to_nine" program
  dut._log.info("Test")
  output = run_test_program(dut, "sum_one_to_nine.bin")
  assert output[4] == 1+2+3+4+5+6+7+8+9
