# FIRA_code_2023
# International Intelligent RoboSports Cup 2023 - FIRA Air Autonomous Race_Pro and FIRA Air Emergency Service Indoor
# 🤖 FIRA Robot Motion & Perception Modules

# 本專案包含 FIRA（Federation of International Robot-soccer Association）2個比賽項目之無人機的感知與控制模組程式碼。

# 核心模組：

- `pers.py`: 處理無人機的感知資訊（如球的位置、對手、隊友等）
- `move.py`: 控制無人機的自主導航邏輯與移動

---

## 📁 專案結構

```
├── fira_1_pers.py    # FIRA Air Autonomous Race_Pro 比賽項目的無人機感知模組
├── fira_1_move.py    # FIRA Air Autonomous Race_Pro 比賽項目的無人機移動控制
├── fira_2_pers.py    # FIRA Air Emergency Service Indoor 比賽項目的無人機感知模組
├── fira_2_move.py    # FIRA Air Emergency Service Indoor 比賽項目的無人機移動控制
```

---

## ⚙️ 功能介紹

### `*_pers.py` – 無人機感知處理模組

- 根據無人機鏡頭回傳資訊分析場上物件相對位置
- 判斷無人機的狀態與場上策略
- 將感知結果供決策模組或移動模組使用

### `*_move.py` – 無人機移動控制模組

- 控制無人機依據目標進行自主導航
- 使用向量運算與角度修正計算移動速度、移動方向、機身旋轉角度、鏡頭角度
- 支援定點移動、目標跟隨、自主修正偏差角度

---

## 🔧 安裝與套件

本專案主要使用以下 Python 套件：

- `numpy`
- `math`

安裝依賴：

```bash
pip install numpy
```

---

## 🚀 使用方式

可以依照需求在主控腳本中匯入下列模組：

```python
from fira_1_move import move_to_ball
from fira_1_pers import get_robot_state
```

根據自身需求，分別呼叫 FIRA 1 與 FIRA 2 的行為模組。

---

## 📌 備註與擴充

- 模組皆為函式導向，便於整合於大型策略決策系統中。
- 若需搭配實際無人機與影像感知模組，請將感知資料格式與模組格式統一。

---

## 👨‍💻 作者資訊

此專案由國立政治大學StarLab團隊研發與設計，主要用於FIRA比賽中的決策控制、導航運動與場地感知系統開發。
