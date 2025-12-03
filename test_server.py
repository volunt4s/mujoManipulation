import mujoco
import mujoco.viewer
import numpy as np
import socket
import threading
import time

# --- (사용자님의 기존 코드 import) ---
from mujoManipulation.controller.PID import PIDController
from mujoManipulation.robot.Panda import FrankaPanda

# --- 소켓 및 동기화 설정 ---
HOST = '127.0.0.1'  # Localhost
PORT = 65432        # 포트 번호
BUFFER_SIZE = 1024  
# 뮤텍스: 두 스레드가 desired_joint 변수에 동시에 접근하는 것을 방지
data_lock = threading.Lock() 

# --- 전역 변수 ---
# 초기 목표 위치는 idle_pose로 설정
panda_model = FrankaPanda(xml_path="/Users/hong/Desktop/github/mujoManipulation/mujoManipulation/assets/panda/franka_panda.xml")
desired_joint_global = panda_model.idle_pose.copy() 

def socket_server_thread(host, port):
    global desired_joint_global
    
    # 서버 소켓을 열고 닫는 부분 (이것은 한 번만 실행)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # 이전에 사용된 주소 재사용 옵션 설정 (바로 재시작 가능)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        s.bind((host, port))
        s.listen()
        print(f"✅ 서버 시작. {host}:{port}에서 연결 대기 중...")
        
        # ⭐ 핵심 수정: 외부 루프를 추가하여 연결이 끊어져도 계속 대기 ⭐
        while True: 
            try:
                # 새로운 연결이 올 때까지 여기서 대기
                conn, addr = s.accept()
                print(f"🔗 클라이언트 연결됨: {addr}")
                
                with conn:
                    while True:
                        data = conn.recv(BUFFER_SIZE)
                        if not data:
                            print(f"❌ 클라이언트 {addr} 연결 종료.")
                            break # 내부 루프 탈출
                        
                        # (이전과 동일한 데이터 처리 로직)
                        try:
                            joint_str = data.decode().split(',')
                            new_command = np.array([float(j) for j in joint_str])
                            
                            with data_lock:
                                if len(new_command) == len(desired_joint_global):
                                    desired_joint_global = new_command
                                
                        except Exception:
                            pass # 오류 무시 또는 로깅
                        
                        conn.sendall(b"ACK") 
                        
            except Exception as e:
                print(f"서버 오류 발생: {e}")
                time.sleep(1) # 오류 발생 시 잠시 대기 후 재시도

                
# --- MuJoCo 메인 루프 ---
def mujoco_main():
    global desired_joint_global
    m = panda_model.mj_model
    d = panda_model.mj_data

    controller = PIDController(K_p = 1000, K_i = 0.1, K_d = 50, dt = 0.001, tau = 0.002, robot_model=panda_model)

    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            
            # -----------------------------------------------------
            # 1. READ & UPDATE (제어)
            # -----------------------------------------------------
            # 뮤텍스 락을 걸고 전역 변수에서 목표 값을 읽어옴 (안정성 확보)
            with data_lock:
                current_desired_joint = desired_joint_global.copy()
            
            ut = controller.update(current_desired_joint, d.qpos)
            
            # -----------------------------------------------------
            # 2. WRITE & STEP
            # -----------------------------------------------------
            d.ctrl = ut
            mujoco.mj_step(m, d)
            viewer.sync()

# --- 실행 ---
if __name__ == "__main__":
    # 소켓 서버를 별도의 스레드에서 실행
    server_thread = threading.Thread(target=socket_server_thread, args=(HOST, PORT))
    server_thread.daemon = True
    server_thread.start()
    
    # MuJoCo 메인 루프 실행
    mujoco_main()