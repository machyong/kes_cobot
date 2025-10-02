#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ChatServer(Node):
    def __init__(self):
        super().__init__('chat_server')
        # 서비스 이름: 'chat_service'
        self.srv = self.create_service(Trigger, 'chat_service', self.handle_chat)
        self.get_logger().info("chat_service 준비 완료. (서비스 호출 시 터미널에서 입력받습니다)")

    def handle_chat(self, request, response):
        """
        서비스가 들어오면 터미널에서 입력을 받아 그 값을 response.message로 반환.
        """
        try:
            # 터미널에 프롬프트 출력 (ros 로그는 버퍼링될 수 있으니 print 사용)
            print("\n[chat_service] 응답으로 보낼 문장을 입력하세요 (Ctrl+C로 취소): ", end="", flush=True)
            user_text = input()

            if user_text.strip() == "":
                response.success = False
                response.message = "(빈 문자열)"
                self.get_logger().warn("빈 문자열이 입력되어 실패로 응답합니다.")
            else:
                response.success = True
                response.message = user_text
                self.get_logger().info(f"입력 수신 → '{user_text}'")
        except (KeyboardInterrupt, EOFError):
            response.success = False
            response.message = "(입력 취소)"
            self.get_logger().warn("사용자가 입력을 취소했습니다.")
        except Exception as e:
            response.success = False
            response.message = f"(예외 발생: {e})"
            self.get_logger().error(f"입력 처리 중 예외: {e}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ChatServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
