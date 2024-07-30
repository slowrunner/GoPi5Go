#!/usr/bin/env python3
import time

from qiskit import transpile
from qiskit.circuit.library import RealAmplitudes
from qiskit.quantum_info import SparsePauliOp
from qiskit_aer import AerSimulator

sim = AerSimulator()
# --------------------------
# Simulating using estimator
#---------------------------
from qiskit_aer.primitives import EstimatorV2

psi1 = transpile(RealAmplitudes(num_qubits=2, reps=2), sim, optimization_level=0)
psi2 = transpile(RealAmplitudes(num_qubits=2, reps=3), sim, optimization_level=0)

H1 = SparsePauliOp.from_list([("II", 1), ("IZ", 2), ("XI", 3)])
H2 = SparsePauliOp.from_list([("IZ", 1)])
H3 = SparsePauliOp.from_list([("ZI", 1), ("ZZ", 1)])

theta1 = [0, 1, 1, 2, 3, 5]
theta2 = [0, 1, 1, 2, 3, 5, 8, 13]
theta3 = [1, 2, 3, 4, 5, 6]

estimator = EstimatorV2()

t1 = time.perf_counter()
# calculate [ [<psi1(theta1)|H1|psi1(theta1)>,
#              <psi1(theta3)|H3|psi1(theta3)>],
#             [<psi2(theta2)|H2|psi2(theta2)>] ]
job = estimator.run(
    [
        (psi1, [H1, H3], [theta1, theta3]),
        (psi2, H2, theta2)
    ],
    precision=0.01
)
t2 = time.perf_counter()
result = job.result()
print("\nSimulating Two 2-qubit circuits")
print(f"expectation values : psi1 = {result[0].data.evs}, psi2 = {result[1].data.evs}")
print("time: {:.2f} ms".format((t2-t1)*1000))

# --------------------------
# Simulating using sampler
# --------------------------
from qiskit_aer.primitives import SamplerV2
from qiskit import QuantumCircuit

# create a Bell circuit
bell = QuantumCircuit(2)
bell.h(0)
bell.cx(0, 1)
bell.measure_all()

# create two parameterized circuits
pqc = RealAmplitudes(num_qubits=2, reps=2)
pqc.measure_all()
pqc = transpile(pqc, sim, optimization_level=0)
pqc2 = RealAmplitudes(num_qubits=2, reps=3)
pqc2.measure_all()
pqc2 = transpile(pqc2, sim, optimization_level=0)

theta1 = [0, 1, 1, 2, 3, 5]
theta2 = [0, 1, 2, 3, 4, 5, 6, 7]

# initialization of the sampler
sampler = SamplerV2()

# collect 128 shots from the Bell circuit
t1=time.perf_counter()
job = sampler.run([bell], shots=128)
t2=time.perf_counter()
job_result = job.result()
print("\nSimulating 128 shots of a 2-qubit circuit")
print(f"counts for Bell circuit : {job_result[0].data.meas.get_counts()}")
print("time: {:.2f} ms".format((t2-t1)*1000))

# run a sampler job on the parameterized circuits
t1=time.perf_counter()
job2 = sampler.run([(pqc, theta1), (pqc2, theta2)])
t2=time.perf_counter()
job_result = job2.result()
print("\nSimulating a 2-qubit circuit")
print(f"counts for parameterized circuit : {job_result[0].data.meas.get_counts()}")
print("time: {:.2f} ms".format((t2-t1)*1000))

