## 🛠️ Setup
conda create -n vis_rl python=3.10
conda activate vis_rl
git clone https://github.com/sanghyunryoo/VisualRL_POSTECH_Pendulum.git
cd VisualRL_POSTECH_Pendulum/
pip install -r requirements.txt

## 🚀 Training
python main_rgb.py

## 🎯 Evaluation
python main_rgb.py --eval True
