# https://pypi.org/project/stockfish/
from stockfish import Stockfish

stockfish = Stockfish("/usr/games/stockfish")

stockfish = Stockfish(parameters={
    "Write Debug Log": "false",
    "Contempt": 0,
    "Min Split Depth": 0,
    "Threads": 1,
    "Ponder": "false",
    "Hash": 16,
    "MultiPV": 1,
    "Skill Level": 20,
    "Move Overhead": 30,
    "Minimum Thinking Time": 20,
    "Slow Mover": 80,
    "UCI_Chess960": "false",
})

stockfish.set_position(["e2e4", "e7e6"])

print(stockfish.is_move_correct('a2a3'))

print(stockfish.get_best_move())

print(stockfish.get_board_visual())