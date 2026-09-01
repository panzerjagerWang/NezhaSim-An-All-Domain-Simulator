#!/usr/bin/env python3
#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
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

        # Window settings - key step: set a transparent background
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
        """Load frames with FRESH background to prevent ghosting (independent-frame mode)"""
        try:
            img = Image.open(self.gif_path)

            self.frames = []
            self.frame_delays = []

            frame_iter = 0
            while True:
                try:
                    img.seek(frame_iter)

                    # === Core change: independent-frame mode ===
                    # 1. Create a brand-new black background for every frame
                    # This forcibly clears anything left over from the previous frame, fixing the ghosting issue
                    # (0, 0, 0, 255) represents opaque pure black
                    new_frame = Image.new("RGBA", img.size, (0, 0, 0, 255))

                    # 2. Get the current GIF frame
                    current_part = img.copy().convert('RGBA')

                    # 3. Paste the current GIF frame onto the brand-new black background
                    new_frame.paste(current_part, (0, 0), current_part)

                    # 4. Save this frame
                    self.frames.append(ImageTk.PhotoImage(new_frame))

                    # 5. Get the duration
                    duration = img.info.get('duration', 50)
                    self.frame_delays.append(duration)

                    frame_iter += 1
                except EOFError:
                    break  # End of frames

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
            # Stop the animation callback
            # Although is_running is set to False, cancelling the callback adds an extra safeguard
            # (it can be skipped in a simple script; Tkinter destroy handles it automatically)

            # Hide and destroy directly; do not use an alpha fade, which tends to cause ghosting on Linux
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
