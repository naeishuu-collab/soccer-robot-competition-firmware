"""カメラ映像の確認用起動スクリプト。UART送信を止め、現行検出処理の画面表示だけ行う。
実処理はgoal_wall_navigation.main_loopへ委譲するため、校正ファイルもカメラ上に必要。
"""
from goal_wall_navigation import main_loop

SEND_UART = False

if __name__ == "__main__":
    main_loop(send_uart=SEND_UART, display=True)
