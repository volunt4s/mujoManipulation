import socket
import numpy as np
import time

# --- 서버와 동일한 설정 ---
HOST = '127.0.0.1'
PORT = 65432
BUFFER_SIZE = 1024

# --- 전송할 목표 관절 위치 목록 (예시) ---
# Panda 로봇의 7개 관절에 대한 목표 위치
commands = [
    # 1. 초기 위치 (idle pose 근처)
    np.random.uniform(-2, 2, 9),
    
    # 2. 앞으로 쭉 편 위치
    np.random.uniform(-2, 2, 9),
    
    # 3. 위로 든 위치
    np.random.uniform(-2, 2, 9),
    
    # 4. 다시 초기 위치
    np.random.uniform(-2, 2, 9)
]


# --- 클라이언트 실행 ---
try:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print(f"✅ 서버 {HOST}:{PORT}에 연결 성공")

        print("목표 관절 위치 명령 전송 시작...")
        
        for i, target_joint_pos in enumerate(commands):
            print(f"--- 목표 위치 {i+1} 전송 ---")
            
            # 1. 명령을 문자열로 변환 (0.0, -0.785, ... 형태)
            command_str = ','.join(map(str, target_joint_pos))

            # 2. 데이터 전송
            s.sendall(command_str.encode())
            
            # 3. 서버로부터 응답(ACK) 대기 (데이터 동기화)
            response = s.recv(BUFFER_SIZE)
            if response == b"ACK":
                print(f"   -> 전송 성공. {target_joint_pos[:3]}...")
            else:
                print("   -> 서버 응답 오류")
                
            time.sleep(3) # 3초 간격으로 다음 목표 전송

except ConnectionRefusedError:
    print("🚨 오류: MuJoCo 서버(mujoco_server.py)가 실행 중인지, 포트가 열려 있는지 확인하세요.")
except Exception as e:
    print(f"🚨 클라이언트 오류 발생: {e}")