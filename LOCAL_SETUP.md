# Local Environment Setup

This repository is intended to be shared as source code, not as a committed virtual environment.
The steps below let each collaborator set up a clean local environment from this branch.

## 1) Clone the branch

```bash
git clone https://github.com/parkmoses02/equilibrium.git
cd equilibrium
git checkout <your-branch-name>
```

If you already have the repository, fetch the latest branch changes and switch to the branch you want to work on.

## 2) Python setup

Create a virtual environment in the repository root.

```bash
python -m venv .venv
```

Activate it:

Windows PowerShell:

```powershell
.\.venv\Scripts\Activate.ps1
```

Windows Command Prompt:

```bat
.\.venv\Scripts\activate.bat
```

macOS / Linux:

```bash
source .venv/bin/activate
```

Install dependencies:

```bash
pip install -r requirements.txt
```

## 3) Run the Python template

```bash
python dynamics_template.py
```

The script prints:
- `M(q)`
- `C(q, qdot)`
- `G(q)`
- `qddot`
- linearized `A` and `B`

## 4) MATLAB setup

No extra Python packages are needed for MATLAB.
Open `dynamics_template.m` in MATLAB and call the function with your parameter struct.

Example:

```matlab
p = struct('m_total',2.0,'A',0.25,'B',0.10,'Cc',0.05, ...
           'I_link1',0.30,'I_link2',0.12,'k1',4.2,'k2',1.8);
q = [0; 0.05; -0.03];
qdot = [0.1; 0; 0];
tau = [0; 0; 0];
out = dynamics_template(q, qdot, tau, p);
```

## 5) Files that should stay local

Do not commit the following local-only files or folders:
- `.venv/`
- `.vscode/`
- Python cache files such as `__pycache__/`

## 6) Notes

- Keep the parameter values consistent between Python and MATLAB.
- If you change the model coefficients, update both the code and the documentation.
- For shared development, commit source files and docs only.
