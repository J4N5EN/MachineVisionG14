# MachineVisionDrone
Exam handin by Andreas Jansen and Daniel Berntsen

# For generating aruco markers
https://chev.me/arucogen/

# Set up env:
python -m venv env
.\env\Scripts\activate
pip install -r requirements.txt


# How to run:
1. Connect to drone over WiFi
    - Turn drone on
    - Once it is blinking you can connect to its open network, usually named something with Tello
2. Run the code, make sure the video stream is functioning properly (updating)
3. To search for a landing pad:
    - Press the number of pad you want, the code by default supports 0-3, to add more keybinds for ids:
        - find "key_functions" in main.py.
        - add "k": lambda: set_search_id(n)," to it, where k is the keybind you want and n is the id (max 250)
    - Press "s" to enable search mode
    - Press "x" for take-off. The code will do the rest.
4. If you want to search for another pad do the steps in 3. again, except press "d" before.
    "d" disables and resetts all the variables needed for searching

"z" makes the drone land. If it is busy doing something you might have to press it more than once, usually works by just pressing it once.


# Other:
ChatGPT was used during this project. Uses include:
Explaining topics for learning,
Optimizing code (finding faster ways to run code),
and making the code more readable, includes giving names to functions and reorganizing.
More detailed explanation regarding use of ChatGPT in the report.
