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
# Install Latest Qiskit Version

REF: https://github.com/Qiskit/qiskit/tree/main

```
python3 -m venv qiskit1 --prompt qiskit1
cd qiskit1
source bin/activate
pip install qiskit
```

- pip show qiskit
```
(qiskit1) pi@GoPi5Go:~/GoPi5Go/systests/qvm/qiskit1 $ pip show qiskit
Name: qiskit
Version: 1.1.1
Summary: An open-source SDK for working with quantum computers at the level of extended quantum circuits, operators, and primitives.
Home-page: 
Author: 
Author-email: Qiskit Development Team <qiskit@us.ibm.com>
License: Apache 2.0
Location: /home/pi/GoPi5Go/systests/qvm/qiskit1/lib/python3.11/site-packages
Requires: dill, numpy, python-dateutil, rustworkx, scipy, stevedore, symengine, sympy, typing-extensions
```


- 3qubitGHZ.py
```
(qiskit1) pi@GoPi5Go:~/GoPi5Go/systests/qvm/qiskit1/examples $ ./3qubitGHZ.py 
 > Quasi probability distribution: [{0: np.float64(0.528), 7: np.float64(0.472)}]
Execution Time: 1.70 ms

 > Expectation values: [4.]
Execution Time: 0.69 ms
```

### v1.1.1 appears to be roughly 55% faster than v0.14

- wget https://github.com/Qiskit/qiskit/raw/main/examples/python/qft.py

```
(qiskit1) pi@GoPi5Go:~/GoPi5Go/systests/qvm/qiskit1/examples $ ./qft.py 
     ┌───┐ ┌───────┐  ░ ┌───┐                                      ░ ┌─┐      
q_0: ┤ H ├─┤ P(-π) ├──░─┤ H ├─■─────────────■──────────────────────░─┤M├──────
     ├───┤┌┴───────┴┐ ░ └───┘ │P(π/2) ┌───┐ │                      ░ └╥┘┌─┐   
q_1: ┤ H ├┤ P(-π/2) ├─░───────■───────┤ H ├─┼────────■─────────────░──╫─┤M├───
     ├───┤├─────────┤ ░               └───┘ │P(π/4)  │P(π/2) ┌───┐ ░  ║ └╥┘┌─┐
q_2: ┤ H ├┤ P(-π/4) ├─░─────────────────────■────────■───────┤ H ├─░──╫──╫─┤M├
     └───┘└─────────┘ ░                                      └───┘ ░  ║  ║ └╥┘
q_3: ─────────────────░────────────────────────────────────────────░──╫──╫──╫─
                      ░                                            ░  ║  ║  ║ 
q_4: ─────────────────░────────────────────────────────────────────░──╫──╫──╫─
                      ░                                            ░  ║  ║  ║ 
c: 5/═════════════════════════════════════════════════════════════════╩══╩══╩═
                                                                      0  1  2 
     ┌───┐ ┌───────┐  ░ ┌───┐                                                                      ░ ┌─┐         
q_0: ┤ H ├─┤ P(-π) ├──░─┤ H ├─■─────────────■─────────────────■────────────────────────────────────░─┤M├─────────
     ├───┤┌┴───────┴┐ ░ └───┘ │P(π/2) ┌───┐ │                 │                                    ░ └╥┘┌─┐      
q_1: ┤ H ├┤ P(-π/2) ├─░───────■───────┤ H ├─┼────────■────────┼─────────────■──────────────────────░──╫─┤M├──────
     ├───┤├─────────┤ ░               └───┘ │P(π/4)  │P(π/2)  │       ┌───┐ │                      ░  ║ └╥┘┌─┐   
q_2: ┤ H ├┤ P(-π/4) ├─░─────────────────────■────────■────────┼───────┤ H ├─┼────────■─────────────░──╫──╫─┤M├───
     ├───┤├─────────┤ ░                                       │P(π/8) └───┘ │P(π/4)  │P(π/2) ┌───┐ ░  ║  ║ └╥┘┌─┐
q_3: ┤ H ├┤ P(-π/8) ├─░───────────────────────────────────────■─────────────■────────■───────┤ H ├─░──╫──╫──╫─┤M├
     └───┘└─────────┘ ░                                                                      └───┘ ░  ║  ║  ║ └╥┘
q_4: ─────────────────░────────────────────────────────────────────────────────────────────────────░──╫──╫──╫──╫─
                      ░                                                                            ░  ║  ║  ║  ║ 
c: 5/═════════════════════════════════════════════════════════════════════════════════════════════════╩══╩══╩══╩═
                                                                                                      0  1  2  3 
     ┌───┐ ┌───────┐   ░ ┌───┐                                                                                                                ░ ┌─┐            
q_0: ┤ H ├─┤ P(-π) ├───░─┤ H ├─■─────────────■─────────────────■──────────────────────■───────────────────────────────────────────────────────░─┤M├────────────
     ├───┤┌┴───────┴┐  ░ └───┘ │P(π/2) ┌───┐ │                 │                      │                                                       ░ └╥┘┌─┐         
q_1: ┤ H ├┤ P(-π/2) ├──░───────■───────┤ H ├─┼────────■────────┼─────────────■────────┼──────────────────■────────────────────────────────────░──╫─┤M├─────────
     ├───┤├─────────┤  ░               └───┘ │P(π/4)  │P(π/2)  │       ┌───┐ │        │                  │                                    ░  ║ └╥┘┌─┐      
q_2: ┤ H ├┤ P(-π/4) ├──░─────────────────────■────────■────────┼───────┤ H ├─┼────────┼─────────■────────┼─────────────■──────────────────────░──╫──╫─┤M├──────
     ├───┤├─────────┤  ░                                       │P(π/8) └───┘ │P(π/4)  │         │P(π/2)  │       ┌───┐ │                      ░  ║  ║ └╥┘┌─┐   
q_3: ┤ H ├┤ P(-π/8) ├──░───────────────────────────────────────■─────────────■────────┼─────────■────────┼───────┤ H ├─┼────────■─────────────░──╫──╫──╫─┤M├───
     ├───┤├─────────┴┐ ░                                                              │P(π/16)           │P(π/8) └───┘ │P(π/4)  │P(π/2) ┌───┐ ░  ║  ║  ║ └╥┘┌─┐
q_4: ┤ H ├┤ P(-π/16) ├─░──────────────────────────────────────────────────────────────■──────────────────■─────────────■────────■───────┤ H ├─░──╫──╫──╫──╫─┤M├
     └───┘└──────────┘ ░                                                                                                                └───┘ ░  ║  ║  ║  ║ └╥┘
c: 5/════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════╩══╩══╩══╩══╩═
                                                                                                                                                 0  1  2  3  4 
Basic simulator
{'00001': 1024}
{'00001': 1024}
{'00001': 1024}
```

