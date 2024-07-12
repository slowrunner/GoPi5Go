#!/usr/bin/env python3

"""
Written by Alauddin Maulana Hirzan
https://brew.bsd.cafe/maulanahirzan/Python-Useful-Scripts/raw/branch/main/MIPS-Benchmark/MIPSBenchmark.py
"""
from time import time

class MIPSBenchmark:
    def __init__(self, iters: int):
        self.iters = iters
        self.start_time = 0
        self.end_time = 0
        self.elapsed_time = 0
        self.total_instructions = 0
        self.mips = 0

    def ips_operation(self):
        # Run basic 7 ALU operations
        _ = 5 + 5
        _ = 5 - 5
        _ = 5 * 5
        _ = 5 // 5
        _ = True & True
        _ = True | False
        _ = False ^ False

    def calculate(self):
        self.elapsed_time = self.end_time - self.start_time
        self.total_instructions = self.iters*7
        self.mips = (self.total_instructions / self.elapsed_time) 
        print(f"=> Executed {self.total_instructions} within {self.elapsed_time:.2f}")
        print(f"=> Result : {self.mips:.2f} IPS / {self.mips/1e6:.2f} MIPS")

    def run(self):
        print("# MIPS CPU Benchmark #")
        print("# Running 7 Basic ALU Ops #")
        print("# Please Wait #")
        
        self.start_time = time()
        for i in range(1, self.iters+1):
            self.ips_operation()
        self.end_time = time()

        self.calculate()


def main():
    iters = 1000000 # Minimum 1,000,000 times
    bench = MIPSBenchmark(iters=iters)
    bench.run()


main()
