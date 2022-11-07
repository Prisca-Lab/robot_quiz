
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
        self.row_df = row
        self.question = row[["DOMANDA"]]
        self.available_answers = row[["RISP1","RISP2","RISP3","RISP4"]]
        self.correct_answer_idx = row[["CORRETTA"]] # store the name of the column that contains the right answer
        
        self.done = False
        

    def check(self, answer) -> bool:
        """check if the answer is correct
        """
        self.done = True
        if answer == self.correct_answer:
            return True
        else:
            return False

    def get_hinted(self) -> str:
        """return a question that contains a correct answer and a wrong answer
        """

        correct_answer = self.row_df[self.correct_answer_idx.values]
        wrong_answers = self.available_answers.drop(self.correct_answer_idx)
        w1 = wrong_answers.iloc[[0]]
        hinted = pd.concat([correct_answer.T, w1])
        return hinted
        