
from pathlib import Path
import pandas as pd
from itertools import cycle

INTENT_OPTIONS = ['risposta_uno', 'risposta_due',
                  'risposta_tre', 'risposta_quattro']


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
    row: pandas dataframe of a question
    type: question | hinted
    """
    def __init__(self, row, type="question") -> None:
        self.row_df = row
        self.type = type
        self.question = row[["DOMANDA"]]
        self.available_answers = row[INTENT_OPTIONS]
        self.correct_answer_idx = row[["CORRETTA"]] # store the name of the column that contains the right answer
        
        self.done = False
        

    def check(self, answer) -> bool:
        """check if the answer is correct
        """
        self.done = True
        if answer == self.correct_answer_idx.values[0]:
            return True
        else:
            return False

    def get_hinted(self) -> str:
        """return a question that contains a correct answer and a wrong answer
        """
        hinted = self # is needed to maintain same id and not create a new object

        correct_answer = self.row_df[self.correct_answer_idx.values]
        wrong_answers = self.available_answers.drop(self.correct_answer_idx)
        w1 = wrong_answers.iloc[[0]]
        hinted_answers = pd.concat([correct_answer.T, w1])
        hinted.available_answers = hinted_answers
        hinted.type = "hinted"
        
        return hinted
        
class Personality:
    def __init__(self, type: str) -> None:
        """
        type (str): AGREEABLENESS | ANTAGONIST
        """
        self.name = type
        self.on_hint = load_file("on_hint_personality.csv")
        self.feedback = load_file("feedback_personality.csv")

        fb = self.feedback[self.feedback["NOME_TIPO"] == type]
        
        # use cycle to iterate over the same list
        self.hint = cycle(self.on_hint[self.on_hint["NOME_TIPO"] == type].values.tolist())
        self.positive_fb = cycle(fb["POSITIVE_FB"].tolist())
        self.negative_fb = cycle(fb["NEGATIVE_FB"].tolist())

    def get_positive(self):
        return next(self.positive_fb)

    def get_negative(self):
        return next(self.negative_fb)

    def get_hint(self):
        return next(self.hint)[1] # [0] is the name of the personality type; [1] is the feedback sentence


if __name__ == '__main__':
    P = Personality("AGREEABLENESS")
    print(P.get_positive())