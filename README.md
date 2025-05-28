# Misty Coqui-TTS Node

Basic instructions to get neural TTS running on Misty.

## 1. Prerequisites

* Ubuntu with ROS 2 Jazzy installed
* Python 3.12

## 2. Python Setup

```bash
cd ~(workspace)
python3 -m venv misty
source misty/bin/activate
pip install --upgrade pip
pip install coqui-tts[server] soundfile pydub
```

## 3. ROS Workspace

```bash
cd ~/misty_ws
source /opt/ros/jazzy/setup.bash
source ~/misty/bin/activate
colcon build --cmake-args -DPYTHON_EXECUTABLE=$(which python3)
source install/setup.bash
```

## 4. Run the Node

```bash
ros2 run misty_interaction tts_node \
  --ros-args \
    -p misty_ip:=<MISTY_IP> \
    -p tts_model:=tts_models/en/ljspeech/vits \
    -p cache_and_save:=true \
    -p cache_dir:=~/misty_ws/misty_phrases
```

## 5. Test Speech

In a new terminal (sourced same as above):

```bash
ros2 topic pub /interaction_text std_msgs/msg/String "{data: 'Hello!'}"
```

Misty should speak “Hello!” and save an MP3 in `misty_phrases`.


## re-opening enviroment

source ~/misty/bin/activate

source ~/misty_ws/install/setup.bash

python3 -m misty_interaction.TTSNode


# list of voice indexes

['ED\n', 'p225', 'p226', 'p227', 'p228', 'p229', 'p230', 'p231', 'p232', 'p233', 'p234', 'p236', 'p237', 'p238', 'p239', 'p240', 'p241', 'p243', 'p244', 'p245', 'p246', 'p247', 'p248', 'p249', 'p250', 'p251', 'p252', 'p253', 'p254', 'p255', 'p256', 'p257', 'p258', 'p259', 'p260', 'p261', 'p262', 'p263', 'p264', 'p265', 'p266', 'p267', 'p268', 'p269', 'p270', 'p271', 'p272', 'p273', 'p274', 'p275', 'p276', 'p277', 'p278', 'p279', 'p280', 'p281', 'p282', 'p283', 'p284', 'p285', 'p286', 'p287', 'p288', 'p292', 'p293', 'p294', 'p295', 'p297', 'p298', 'p299', 'p300', 'p301', 'p302', 'p303', 'p304', 'p305', 'p306', 'p307', 'p308', 'p310', 'p311', 'p312', 'p313', 'p314', 'p316', 'p317', 'p318', 'p323', 'p326', 'p329', 'p330', 'p333', 'p334', 'p335', 'p336', 'p339', 'p340', 'p341', 'p343', 'p345', 'p347', 'p351', 'p360', 'p361', 'p362', 'p363', 'p364', 'p374', 'p376']
speaker_idx	VCTK ID	Gender	Notes / accent*
0	p225	F	Southern England
1	p226	M	Scottish
2	p227	F	Northern England
3	p228	M	Northern England
4	p229	M	Southern England
5	p230	M	Northern Ireland
6	p231	M	Irish
7	p232	M	Midlands
8	p233	M	Received-Pronunciation
9	p234	F	RP / London
10	p236	M	Welsh
11	p237	M	Scottish
12	p238	M	Southern England
13	p239	M	Midlands
14	p240	F	RP
15	p241	F	Northern England
16	p243	F	Southern England
17	p244	M	Scottish
18	p245	F	RP
19	p246	M	Southern England