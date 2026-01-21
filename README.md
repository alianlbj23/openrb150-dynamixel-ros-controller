# Server Control Panel

This application provides a graphical user interface (GUI) to control a remote server, allowing users to connect, disconnect, and manage services like SLAM and Localization.

## Prerequisites

*   Python 3.x
*   The Python packages listed in `requirements.txt`.

## Setup

1.  **Clone the repository or download the files.**
2.  **Navigate to the project directory:**
    ```bash
    cd /path/to/your/project/pros_web_client
    ```
3.  **Create a virtual environment (recommended):**
    ```bash
    python3 -m venv venv
    source venv/bin/activate  # On Windows use `venv\Scripts\activate`
    ```
4.  **Install the required packages:**
    ```bash
    pip install -r requirements.txt
    ```

## How to Run

1.  **Ensure your virtual environment is activated** (if you created one).
2.  **Run the `main.py` script:**
    ```bash
    python main.py
    ```
    This will launch the Server Control Panel GUI.

## Usage Instructions

The application window allows you to interact with a server that exposes specific API endpoints (expected to be running on port 5000).

### 1. Connecting to the Server

*   **Enter the Server IP:** In the "Server IP" field, type the IP address of the server you want to connect to (e.g., `192.168.0.10`).
*   **Click "Connect":**
    *   The application will attempt to connect to `http://<SERVER_IP>:5000/run-script/star_car`.
    *   If successful, the "Connect" button will change to "Disconnect".
    *   The "Slam", "Localization", and "Reset" buttons will become visible.
    *   The connected IP address will be displayed.
    *   A message will confirm the connection status.
    *   If the connection fails or the server returns an error, a warning or error message will be displayed.

### 2. Using Server Functions (After Connecting)

*   **Slam Button:**
    *   Click "Slam" to start the SLAM service. The application sends a request to `http://<SERVER_IP>:5000/run-script/slam_ydlidar`.
    *   If successful, the button text changes to "Close Slam", and the "Localization" button is disabled.
    *   Click "Close Slam" to stop the SLAM service. The application sends a request to `http://<SERVER_IP>:5000/run-script/slam_ydlidar_stop`. The button text reverts to "Slam", and the "Localization" button is re-enabled.
*   **Localization Button:**
    *   Click "Localization" to start the localization service. The application sends a request to `http://<SERVER_IP>:5000/run-script/localization_ydlidar`.
    *   If successful, the button text changes to "Close Localization", and the "Slam" button is disabled.
    *   Click "Close Localization" to stop the localization service. The application sends a request to `http://<SERVER_IP>:5000/run-script/localization_ydlidar_stop`. The button text reverts to "Localization", and the "Slam" button is re-enabled.
*   **Reset Button:**
    *   Click "Reset" to send stop signals for both SLAM and Localization services simultaneously.
    *   This will also reset the state of the "Slam" and "Localization" buttons in the UI, re-enabling them.

### 3. Disconnecting from the Server

*   **Click "Disconnect":**
    *   The application will immediately update the UI to a disconnected state (hiding function buttons, clearing IP, changing button text back to "Connect").
    *   In the background, it sends a request to `http://<SERVER_IP>:5000/run-script/star_car_stop` to stop the main server service.

### Error Handling

*   If the IP format is invalid, a warning will be shown.
*   If connection or script execution requests fail, an error message detailing the issue will be displayed.

## Dependencies

The application relies on the following Python libraries:

*   `requests`: For making HTTP requests to the server.
*   `PyQt5`: For the graphical user interface.

Refer to `requirements.txt` for specific

## pros_twin
https://drive.google.com/drive/u/2/folders/1AtqUCTJ7k7NK46fRaCSsYnG2Mxc2r-qv

## Server
https://github.com/alianlbj23/pros_web_server
# openrb150-dynamixel-ros-controller
