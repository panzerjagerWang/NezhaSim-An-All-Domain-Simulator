#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tkinter as tk
from PIL import Image, ImageTk
import os
import sys


class SplashScreen:
    def __init__(self):
        # Get parameters
        self.gif_path = rospy.get_param('~gif_path', '')
        self.duration = rospy.get_param('~duration', 5000)

        # Validate GIF path
        if not self.gif_path or not os.path.exists(self.gif_path):
            rospy.logerr(f"GIF file not found: {self.gif_path}")
            sys.exit(1)

        rospy.loginfo(f"Loading splash screen from: {self.gif_path}")

        # Create window
        self.root = tk.Tk()
        self.root.title("NEZHA Simulator")

        # Window settings - 关键：设置透明背景
        self.root.overrideredirect(True)
        self.root.attributes('-topmost', True)
        self.root.configure(bg='black')

        # Load GIF first to get size
        self.frames = []
        self.frame_delays = []
        self.load_gif()

        # Create canvas for better rendering
        if self.frames:
            first_frame = self.frames[0]
            width = first_frame.width()
            height = first_frame.height()

            # Use Canvas instead of Label for cleaner rendering
            self.canvas = tk.Canvas(
                self.root,
                width=width,
                height=height,
                bg='black',
                highlightthickness=0,
                bd=0
            )
            self.canvas.pack()

            # Create image on canvas
            self.image_id = self.canvas.create_image(
                width // 2, height // 2,
                image=self.frames[0],
                anchor='center'
            )

        # Center window
        self.center_window()

        # Make sure window is visible
        self.root.deiconify()
        self.root.lift()
        self.root.focus_force()

        # Start animation
        self.is_running = True
        self.current_frame = 0
        self.animate()

        # Schedule close
        self.root.after(self.duration, self.close)

        rospy.loginfo(f"Splash screen displayed with {len(self.frames)} frames")

    def load_gif(self):
        """Load frames with FRESH background to prevent ghosting (独立帧模式)"""
        try:
            img = Image.open(self.gif_path)

            self.frames = []
            self.frame_delays = []

            frame_iter = 0
            while True:
                try:
                    img.seek(frame_iter)

                    # === 核心修改：独立帧模式 ===
                    # 1. 每一帧都创建一个全新的黑色背景
                    # 这样可以强制清除上一帧留下的任何内容，解决重影问题
                    # (0, 0, 0, 255) 代表不透明的纯黑色
                    new_frame = Image.new("RGBA", img.size, (0, 0, 0, 255))

                    # 2. 获取当前 GIF 帧
                    current_part = img.copy().convert('RGBA')

                    # 3. 将当前 GIF 帧贴到全新的黑底上
                    new_frame.paste(current_part, (0, 0), current_part)

                    # 4. 保存这一帧
                    self.frames.append(ImageTk.PhotoImage(new_frame))

                    # 5. 获取时长
                    duration = img.info.get('duration', 50)
                    self.frame_delays.append(duration)

                    frame_iter += 1
                except EOFError:
                    break  # 读取结束

            rospy.loginfo(f"Successfully loaded {len(self.frames)} frames (Fresh Mode)")

        except Exception as e:
            rospy.logerr(f"Error loading GIF: {e}")
            import traceback
            traceback.print_exc()
            self.create_fallback()

    def create_fallback(self):
        """Create a simple text splash as fallback"""
        label = tk.Label(
            self.root,
            text="NEZHA\nUnderwater Simulator",
            font=("Arial", 40, "bold"),
            fg="#16C79A",
            bg="#1A1A2E"
        )
        label.pack(padx=100, pady=100)

    def animate(self):
        """Animate the GIF"""
        if not self.is_running or not self.frames:
            return

        try:
            # Update canvas image
            self.canvas.itemconfig(self.image_id, image=self.frames[self.current_frame])

            # Get delay for current frame
            delay = self.frame_delays[self.current_frame] if self.frame_delays else 50

            # Move to next frame
            self.current_frame = (self.current_frame + 1) % len(self.frames)

            # Schedule next frame
            self.root.after(delay, self.animate)

        except Exception as e:
            rospy.logwarn(f"Animation error: {e}")

    def center_window(self):
        """Center window on screen"""
        self.root.update_idletasks()
        width = self.root.winfo_width()
        height = self.root.winfo_height()
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        x = (screen_width - width) // 2
        y = (screen_height - height) // 2
        self.root.geometry(f'+{x}+{y}')

    def close(self):
        """Close splash screen immediately to avoid Linux alpha bugs"""
        if not self.is_running:
            return

        rospy.loginfo("Closing splash screen")
        self.is_running = False

        try:
            # 停止动画回调
            # 虽然 is_running 设为 False 了，但为了保险可以取消回调
            # (在简单脚本中不做也行，Tkinter destroy 会自动处理)

            # 直接隐藏并销毁，不要做 alpha 渐变，Linux 下容易出重影
            self.root.withdraw()
            self.root.update()

            self.root.quit()
            self.root.destroy()

        except Exception as e:
            rospy.logwarn(f"Error during cleanup: {e}")
            try:
                self.root.destroy()
            except:
                pass

    def run(self):
        """Run the main loop"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.close()
        except Exception as e:
            rospy.logerr(f"Error in mainloop: {e}")
            self.close()


def main():
    rospy.init_node('splash_screen', anonymous=False)

    try:
        splash = SplashScreen()
        splash.run()
    except Exception as e:
        rospy.logerr(f"Splash screen error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
