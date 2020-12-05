**Adaptive Control**
---
Applying adaptive control to a 2DOF and 4DOF robot

**Description**
---
The **MATLAB** folder contains the detailed code for
1. Deriving the symbolic form of the manipulator equation.
2. Deriving the Y matrix and a vector for the adaptive controller
3. Run simple ode45 integration + animation on it.

The [**MuJoCo**](MuJoCo/) simulator consists of the xml model file and the python controller file. That’s simply it. If you have a xml model file and a corresponding controller file, it is ready to run a single-complete simulation. The xml model files are all saved in [**model**](MuJoCo/models) folder, and the python controller objects are all saved in [**controllers.py**](MuJoCo/modules/) folder.

**How to use mujoco-py simulation**
---

```
Usage:
  python3 run.py --modelName="2D_model.xml" --simType=1 --runTime=30
  python3 run.py --modelName="2D_model.xml" --simType=2
  python3 run.py --modelName="3D_model.xml" --simType=1 --saveData
  python3 run.py --modelName="3D_model.xml" --simType=2 --saveData --recordVideo --vidRate=0.5
  python3 run.py --modelName="3D_model.xml" --simType=2 --saveData --videoOFF

Options:
  --simType      1: Joint space trajectory tracking task   2: Cartesian space trajectory tracking task
  --saveData     Saving the essential simulations data as .txt file
  --videoOFF     Turning off the video of the mujoco-py simulation
  --recordVideo  Recording the simulation
  --vidRate      Setting the speed of the video that you are saving, e.g., --vidRate=0.5 means the video is half the speed of original

```

**References**
---
1. [MuJoCo install link](https://www.roboti.us/index.html)
2. [MuJoCo xml model file documentation](http://mujoco.org/book/XMLreference.html)
3. [MuJoCo python controller file documentation](https://openai.github.io/mujoco-py/build/html/index.html)
3. [MuJoCo c++    controller file documentation](http://mujoco.org/book/APIreference.html)
