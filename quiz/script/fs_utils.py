
from pathlib import Path
import pandas as pd


def load_quiz_questions():
    questions_df = load_file("questions.csv")
    questions = []
    for index, row in questions_df.iterrows():
        questions.append(QuizQuestion(row))
    return questions


def load_file(filename):
    file_questions = Path(__file__).parent.parent.joinpath(
        "data/" + filename)
    if 'csv' in filename:
        return pd.read_csv(file_questions, sep=";")
    elif 'txt' in filename:
        return open(file_questions, "r").read()

class QuizQuestion:
    """data structure for a quiz question
    """
    def __init__(self, row) -> None:
        self.question = row[0]
        self.available_answers = [row[1], row[2], row[3], row[4]]
        self.correct_answer = row[5]
        self.done = False

    def check(self, answer) -> bool:
        """check if the answer is correct
        """
        self.done = True
        if answer == self.correct_answer:
            return True
        else:
            return False

