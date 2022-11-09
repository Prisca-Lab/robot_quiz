import pandas as pd

import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import unittest
from script.fs_utils import QuizQuestion, load_quiz_questions
import logging

log = logging.getLogger(__name__)


class TestQuestions(unittest.TestCase):
    def setUp(self):
        log.debug("creating common resources")

    def tearDown(self):
        log.debug("tearing down common resources")

    def test_hinted_question_has_length_2(self):
        q = load_quiz_questions()
        hinted = q[0].get_hinted()
        q[1].get_hinted()
        self.assertTrue(hinted.len == 2)

    def test_hinted_question_has_correct_answer(self):
        q = load_quiz_questions()
        self.assertFalse(q[0].check("risposta_uno"))
        self.assertTrue(q[0].check("risposta_quattro"))
        print()