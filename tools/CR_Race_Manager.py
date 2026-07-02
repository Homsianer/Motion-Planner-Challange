import os
# Suppress Qt internal warnings and multimedia logs
os.environ["QT_LOGGING_RULES"] = "*=false"

# Suppress graphics driver (Mesa/EGL) warnings
os.environ["MESA_DEBUG"] = "silent"
os.environ["LIBGL_DEBUG"] = "quiet"

# (Try to) suppress FFmpeg logs
os.environ["AV_LOG_LEVEL"] = "quiet"

import sys
from pathlib import Path
from PyQt6.QtMultimedia import QMediaPlayer, QAudioOutput
from PyQt6.QtMultimediaWidgets import QVideoWidget
from PyQt6.QtCore import QUrl, Qt, QTimer
import json
from tools.race_analyzer import analyze_race
from PyQt6.QtWidgets import QApplication, QWidget, QGridLayout, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QSizePolicy, QDialog
import math


class RaceManager(QWidget):
    def __init__(self,benchmark:str, video_dir="../videos_for_race/", nr_racers=4):
        super().__init__()
        self.nr_racers = nr_racers
        self.video_dir = video_dir


        self.setWindowFlags(Qt.WindowType.FramelessWindowHint)
        self.showMaximized()

        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(0, 0, 0, 0)


        self.title_bar = QWidget()
        self.title_bar.setStyleSheet("background-color: #0065bd; color: white;")
        self.title_layout = QHBoxLayout(self.title_bar)


        self.window_title_label = QLabel(f"CommonRoad Race at {benchmark}")
        self.window_title_label.setStyleSheet("font-family: 'Arial', sans-serif; font-size: 32px; font-weight: bold; padding: 10px;")


        self.close_button = QPushButton("X")
        self.close_button.setFixedSize(50, 50)
        self.close_button.setStyleSheet("""
            QPushButton {
                font-size: 20px; font-weight: bold; background-color: #ff4c4c; border: none;
            }
            QPushButton:hover { background-color: #ff1c1c; }
        """)
        self.close_button.clicked.connect(self.close)

        self.title_layout.addWidget(self.window_title_label)
        self.title_layout.addStretch()
        self.title_layout.addWidget(self.close_button)


        self.main_layout.addWidget(self.title_bar)



        self.video_layout = QVBoxLayout()
        self.video_layout.setContentsMargins(10, 10, 10, 10)
        self.main_layout.addLayout(self.video_layout)


        self.play_button = QPushButton("START RACE")
        self.play_button.setFixedHeight(80)
        self.play_button.setStyleSheet("""
                    QPushButton {
                        font-family: 'Arial', sans-serif;
                        font-size: 24px; 
                        font-weight: bold; 
                        background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #28a745, stop:1 #218838); 
                        color: white; 
                        border: 3px solid #1e7e34; /* Updated to 3px to match Racing state */
                        border-radius: 15px;
                        padding: 20px;
                    }
                    QPushButton:hover {
                        background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2ebf4f, stop:1 #28a745);
                    }
                """)
        self.play_button.clicked.connect(self.start_race)
        self.main_layout.addWidget(self.play_button)


        self.players = []
        self.finished_players = set()
        self.load_videos()

        self.countdown_value = 3
        self.countdown_timer=None

    def load_videos(self):
        mp4_directory = Path(self.video_dir)
        video_paths = sorted(mp4_directory.glob("*.mp4"))[:self.nr_racers]

        if len(video_paths) < self.nr_racers:
            print(f"Warning: Found {len(video_paths)} videos, but expected {self.nr_racers}.")


        cols = math.ceil(math.sqrt(self.nr_racers))
        total_rows = math.ceil(len(video_paths) / cols)
        row_layouts = []
        for _ in range(total_rows):
            row_box = QHBoxLayout()
            self.video_layout.addLayout(row_box)
            row_layouts.append(row_box)


        for i, path in enumerate(video_paths):
            quadrant_layout = QVBoxLayout()

            title_label = QLabel(f"Racer {i + 1}")
            title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            title_label.setStyleSheet(
                    "font-family: 'Arial', sans-serif; "
                    "font-size: 20px; "
                    "font-weight: bold; "
                    "background-color: #0065bd; "
                    "color: white; "
                    "padding: 5px;"
                )
            title_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Maximum)

            video_widget = QVideoWidget()

            player = QMediaPlayer()
            audio_output = QAudioOutput()

            player.setVideoOutput(video_widget)
            player.setAudioOutput(audio_output)
            player.setSource(QUrl.fromLocalFile(str(path.absolute())))
            player.pause()

            player.positionChanged.connect(
                    lambda pos, p=player: self.handle_position_changed(pos, p)
                )

            self.players.append(player)

            quadrant_layout.addWidget(title_label, stretch=0)
            quadrant_layout.addWidget(video_widget, stretch=1)


            current_row_index = i // cols
            row_layouts[current_row_index].addLayout(quadrant_layout)



    def handle_position_changed(self, position, player):

        duration = player.duration()


        if duration > 0 and position >= (duration - 150):


            if player not in self.finished_players:

                player.pause()
                self.finished_players.add(player)

                # Check if everyone is finished
                if len(self.finished_players) == len(self.players):
                    self.play_button.setEnabled(True)
                    self.play_button.setText("RESTART RACE")
                    self.play_button.setStyleSheet("""
                        QPushButton {
                            font-family: 'Arial', sans-serif;
                            font-size: 24px; 
                            font-weight: bold; 
                            background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #007bf0, stop:1 #0065bd); 
                            color: white; 
                            border: 2px solid #004c8f;
                            border-radius: 15px;
                            padding: 20px;
                        }
                        QPushButton:hover {
                            background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #1a8ffc, stop:1 #007bf0);
                        }
                    """)

                    #self.show_leaderboard()

    def start_race(self):

        self.finished_players.clear()


        for player in self.players:
            player.setPosition(0)


        self.countdown_value = 3


        self.play_button.setEnabled(False)
        self.play_button.setText(str(self.countdown_value))
        self.play_button.setStyleSheet("""
            QPushButton {
                font-family: 'Arial', sans-serif;
                font-size: 64px; 
                font-weight: bold; 
                background-color: #ffaa00; /* Energetic Orange */
                color: white; 
                border-radius: 15px;
            }
        """)


        self.countdown_timer = QTimer(self)
        self.countdown_timer.timeout.connect(self.update_countdown)
        self.countdown_timer.start(1000)

    def update_countdown(self):

        self.countdown_value -= 1

        if self.countdown_value > 0:

            self.play_button.setText(str(self.countdown_value))

        elif self.countdown_value == 0:

            self.play_button.setText("GO!")
            self.play_button.setStyleSheet("""
                QPushButton {
                    font-family: 'Arial', sans-serif;
                    font-size: 64px; 
                    font-weight: bold; 
                    background-color: #ff3333; /* Racing Red */
                    color: white; 
                    border-radius: 15px;
                }
            """)

        else:

            self.countdown_timer.stop()
            self.execute_race()

    def execute_race(self):

        for player in self.players:
            player.play()

        # 2. Return the button to the "RACING..." image state
        self.play_button.setText("RACING...")
        self.play_button.setStyleSheet("""
                    QPushButton {
                        font-family: 'Arial', sans-serif;
                        font-size: 24px; 
                        font-weight: bold; 
                        font-style: italic;
                        background-color: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #555555, stop:1 #333333); 
                        color: #cccccc; 
                        border: 3px dashed #777777;
                        border-radius: 15px;
                        padding: 20px;
                    }
                """)

    # def show_leaderboard(self):
    #
    #     leaderboard_dialog = QDialog(self)
    #     leaderboard_dialog.setWindowTitle("Race Results")
    #     leaderboard_dialog.setFixedSize(400, 300)
    #     leaderboard_dialog.setStyleSheet("background-color: #111111; color: white;")
    #
    #     layout = QVBoxLayout(leaderboard_dialog)
    #     title = QLabel(" LEADERBOARD ")
    #     title.setAlignment(Qt.AlignmentFlag.AlignCenter)
    #     title.setStyleSheet(
    #         "font-family: 'Arial'; font-size: 28px; font-weight: bold; color: #ffaa00; margin-bottom: 10px;")
    #     layout.addWidget(title)
    #
    #     try:
    #         with open("leaderboard.json", "r") as f:
    #             results = json.load(f)
    #
    #         # Sort the results: Numbers first (fastest to slowest), then "DNF" strings
    #         sorted_results = sorted(
    #             results.items(),
    #             key=lambda x: x[1] if isinstance(x[1], (int, float)) else float('inf')
    #         )[:self.nr_racers]
    #         medals = []
    #         for i in range(self.nr_racers+1):
    #             medals.append(f"{i+1}.")
    #
    #
    #         for i, (racer, time) in enumerate(sorted_results):
    #             medal = medals[i]
    #             time_str = f"{time}s" if isinstance(time, (int, float)) else "DNF"
    #
    #             row = QLabel(f"{medal} {racer} with {time_str}")
    #             row.setStyleSheet("font-family: 'Arial'; font-size: 22px; font-weight: bold; padding: 5px;")
    #             layout.addWidget(row)
    #
    #     except FileNotFoundError:
    #         error_lbl = QLabel("No leaderboard data found.\nDid you run analyzer.py?")
    #         error_lbl.setStyleSheet("font-family: 'Arial'; font-size: 18px; color: #ff4c4c;")
    #         error_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
    #         layout.addWidget(error_lbl)
    #
    #     close_btn = QPushButton("CLOSE")
    #     close_btn.setStyleSheet("""
    #                 QPushButton {
    #                     font-family: 'Arial'; font-size: 18px; font-weight: bold;
    #                     background-color: #444444; color: white; border-radius: 10px; padding: 10px;
    #                 }
    #                 QPushButton:hover { background-color: #555555; }
    #             """)
    #     close_btn.clicked.connect(leaderboard_dialog.accept)
    #     layout.addStretch()
    #     layout.addWidget(close_btn)
    #
    #     # Show the popup and wait for the user to close it
    #     leaderboard_dialog.exec()


if __name__ == "__main__":
    nr_racers = 4
    #analyze_race(nr_racers=nr_racers)
    app = QApplication(sys.argv)
    race_manager = RaceManager(benchmark="DEU_Essen-7_3_I-1", nr_racers=nr_racers)
    sys.exit(app.exec())