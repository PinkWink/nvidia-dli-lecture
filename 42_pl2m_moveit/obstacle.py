"""강의 42: 장애물의 자리와 크기. 시뮬레이터(pl2m_moveit.py)와 MoveIt 쪽(add_obstacle.py)이 같은 값을 쓴다.

로봇은 원점에서 +x 를 보고 서 있다. 왼팔이 home 에서 ready 로 갈 때 손끝이 지나는 길 위에 기둥과 컵을 둔다.
좌표는 로봇이 출발한 자리(odom) 기준이고, 단위는 m 이다.
"""
POST_XY = (0.38, 0.15)                 # 기둥의 자리
POST_SIZE = (0.03, 0.03, 0.47)         # 기둥의 크기 (바닥에서 0.47 m 높이)
CUP_RADIUS, CUP_HEIGHT, CUP_MASS = 0.03, 0.09, 0.1     # 기둥 위에 올려 둔 컵 [m, m, kg]
CUP_CENTER = (POST_XY[0], POST_XY[1], POST_SIZE[2] + CUP_HEIGHT / 2)
# MoveIt 에 알려 줄 상자: 기둥과 컵을 한꺼번에 감싼다. 컵보다 옆으로 1 cm, 위로 2 cm 크다
BOX_SIZE = (0.08, 0.08, POST_SIZE[2] + CUP_HEIGHT + 0.02)
BOX_CENTER = (POST_XY[0], POST_XY[1], BOX_SIZE[2] / 2)
