
from pathlib import Path
import pandas as pd
from itertools import cycle
import csv
import os

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
    counter = 1

    def __init__(self, row, type="question") -> None:
        self.row_df = row
        self.type = type
        self.question = row[["DOMANDA"]].values[0]
        self.available_answers = row[INTENT_OPTIONS]
        # store the name of the column that contains the right answer
        self.correct_answer_idx = row[["CORRETTA"]]

        self.id = QuizQuestion.counter
        QuizQuestion.counter += 1

        self.text = self.get_text()
        self.done = False

    def check(self, answer) -> bool:
        """check if the answer is correct
        """
        self.done = True
        if answer == self.correct_answer_idx.values[0]:
            return True
        else:
            return False

    def get_text(self):
        """build the string object containg the question and possible answers
        """
        title_values = zip(self.available_answers.index.to_list(),
                           self.available_answers.values.tolist())
        text_value = ""

        if self.is_hinted:
            values_list = list(title_values)
            text_value = f"{values_list[0][0]}, {values_list[0][1]}. Oppure, {values_list[1][0]}, {values_list[1][1]}."
        else:
            options = ""
            for title, value in title_values:
                options += title + ", " + value + ". "
                text_value = "Domanda numero " + \
                    str(self.id) + ". " + self.question + ". " + options
        text = text_value
        return text

    def get_pre_answer_hint_text(self):
        """suggest the user which answer is correct
        """
        text = "Secondo me la scelta giusta è " + \
            str(self.correct_answer_idx.values[0])
        return text

    def get_hinted(self) -> str:
        """return a question that contains a correct answer and a wrong answer
        """
        hinted = self  # is needed to maintain same id and not create a new object

        correct_answer = self.row_df[self.correct_answer_idx.values]
        wrong_answers = self.available_answers.drop(self.correct_answer_idx)
        w1 = wrong_answers.iloc[[0]]
        hinted_answers = pd.concat(
            [correct_answer.T, w1]) if correct_answer.index[0] == "risposta_uno" else pd.concat([w1, correct_answer.T])
        hinted.available_answers = hinted_answers
        hinted.type = "hinted"
        hinted.text = hinted.get_text()

        return hinted

    @property
    def is_hinted(self):
        if self.type == "hinted":
            return True
        return False

    @property
    def len(self):
        return len(self.available_answers)


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
        self.hint = cycle(
            self.on_hint[self.on_hint["NOME_TIPO"] == type].values.tolist())
        self.positive_fb = cycle(fb["POSITIVE_FB"].tolist())
        self.negative_fb = cycle(fb["NEGATIVE_FB"].tolist())

    def get_positive(self):
        return next(self.positive_fb)

    def get_negative(self):
        return next(self.negative_fb)

    def get_hint(self):
        # [0] is the name of the personality type; [1] is the feedback sentence
        return next(self.hint)[1]


class LogResult:
    def __init__(self, user_id, condition):
        self.user_id = user_id
        self.condition = condition
        self.result = []

        log_folder = "result_quiz_logs"
        name = "user_" + str(self.user_id) + "_condition" + \
            self.condition.name + ".csv"
        self.filename = Path(__file__).parent.parent.joinpath(
            log_folder, name)
        os.makedirs(Path(__file__).parent.parent.joinpath(
            log_folder), exist_ok=True)

        if self.filename.exists():
            raise FileExistsError(
                f"File: {name} already exists, please change user_id")

    def write(self):
        print("saving result to file")
        q = load_quiz_questions()
        data_titles = []
        for i in range(len(q)):
            data_titles.append("question_id_" + str(i+1))
            data_titles.append("correct_" + str(i+1))
            data_titles.append("tentative_" + str(i+1))
            data_titles.append("hinted_" + str(i+1))

        titles = ["user_id", "condition"] + data_titles
        out = [self.user_id, self.condition.name] + self.result
        with open(self.filename, 'w', newline='') as myfile:
            wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
            wr.writerow(titles)
            wr.writerow(out)
        print(f"Saved logs to {self.filename}")


if __name__ == '__main__':
    P = Personality("AGREEABLENESS")
    print(P.get_positive())
