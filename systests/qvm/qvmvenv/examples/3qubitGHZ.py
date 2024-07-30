#!/usr/bin/env python3

import time
import qiskit
from qiskit import transpile
from qiskit_aer import AerSimulator

# Generate 3-qubit GHZ state
circ = qiskit.QuantumCircuit(3)
circ.h(0)
circ.cx(0, 1)
circ.cx(1, 2)
circ.measure_all()

# Construct an ideal simulator
aersim = AerSimulator()
compiled_circuit = transpile(circ,aersim)

# Perform an ideal simulation
t1=time.perf_counter()
result_ideal = aersim.run(compiled_circuit).result()
t2=time.perf_counter()
counts_ideal = result_ideal.get_counts(0)
print('Counts(ideal):', counts_ideal)
print('Execution Time: {:.2f} ms'.format((t2-t1)*1000))
# Counts(ideal): {'000': 493, '111': 531}
