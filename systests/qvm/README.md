# Quantum Virtual Machine and Quantum Computing for GoPiGo3 Robots


### Getting Started

REF: https://qiskit.github.io/qiskit-aer/index.html

```
python3 -m venv qvmvenv --system-site-packages --prompt qvmvenv
cd qvmvenv
source bin/activate
pip install qiskit-aer
```


```
cd GoPi5Go/systests/qvm/qvmvenv/
source bin/activate
cd examples

(after running examples:  ```deactivate```)
```

### first_qisket_circuit.py
```
(qvmvenv) pi@GoPi5Go:~/GoPi5Go/systests/qvm/qvmvenv/examples $ ./first_qisket_circuit.py 

Simulating Two 2-qubit circuits
expectation values : psi1 = [ 1.53564065 -1.10584991], psi2 = 0.17500806267478156
time: 2.51 ms

Simulating 128 shots of a 2-qubit circuit
counts for Bell circuit : {'00': 66, '11': 62}
time: 1.03 ms

Simulating one 2-qubit circuit
counts for parameterized circuit : {'01': 399, '11': 394, '10': 94, '00': 137}
time: 1.49 ms

```

### 3qubitGHZ.py
```
(qvmvenv) pi@GoPi5Go:~/GoPi5Go/systests/qvm/qvmvenv/examples $ ./3qubitGHZ.py 
Counts(ideal): {'111': 507, '000': 517}
Execution Time: 2.65 ms

```

### show_qiskit_version.sh
```
(qvmvenv) pi@GoPi5Go:~/GoPi5Go/systests/qvm/qvmvenv/examples $ ./show_qiskit_version.sh 
Name: qiskit
Version: 1.1.1
Summary: An open-source SDK for working with quantum computers at the level of extended quantum circuits, operators, and primitives.
Home-page: 
Author: 
Author-email: Qiskit Development Team <qiskit@us.ibm.com>
License: Apache 2.0
Location: /home/pi/GoPi5Go/systests/qvm/qvmvenv/lib/python3.11/site-packages
Requires: dill, numpy, python-dateutil, rustworkx, scipy, stevedore, symengine, sympy, typing-extensions
Required-by: qiskit-aer

```
