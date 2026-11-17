"""Robot-independent exact command vocabulary; no ROS, model or motion APIs."""
import math
import re
import unicodedata
from pathlib import Path

import yaml


def normalize(text):
    text = ''.join(c for c in unicodedata.normalize('NFD', text.lower()) if not unicodedata.combining(c))
    return ' '.join(re.findall(r'[a-z0-9]+', text))


class CommandPolicy:
    def __init__(self, path):
        self.config = yaml.safe_load(Path(path).expanduser().read_text(encoding='utf-8'))
        cfg = self.config
        if not isinstance(cfg, dict) or not isinstance(cfg.get('start_paused'), bool):
            raise ValueError('Voice configuration requires start_paused: true/false')
        self.wake_words = cfg.get('wake_words')
        if not isinstance(self.wake_words, list) or not self.wake_words or any(
                not isinstance(w, str) or not normalize(w) for w in self.wake_words):
            raise ValueError('At least one wake word is required')
        for key, low, high in (('max_command_age_s', .5, 5), ('echo_guard_s', .5, 3),
                               ('minimum_word_confidence', .5, 1)):
            value = cfg.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError('Invalid voice setting: ' + key)
        self.phrases, self.grammar = {}, ['[unk]']
        self.responses = cfg.get('responses', {})
        for intent in ('pause', 'follow'):
            synonyms = cfg.get('commands', {}).get(intent)
            responses = self.responses.get(intent)
            if not isinstance(synonyms, list) or not synonyms or not isinstance(responses, list) or not responses:
                raise ValueError('Both command intents require phrases and responses')
            for response in responses:
                if not isinstance(response, str) or not response.strip() or len(response) > 120:
                    raise ValueError('Responses must be short nonempty strings')
                if any(normalize(w) in normalize(response).split() for w in self.wake_words):
                    raise ValueError('Responses must not include wake words')
            for synonym in synonyms:
                if not isinstance(synonym, str) or not normalize(synonym) or len(synonym) > 100:
                    raise ValueError('Invalid command phrase')
                for wake in self.wake_words:
                    phrase = wake.lower() + ' ' + synonym.lower()
                    key = normalize(phrase)
                    if key in self.phrases:
                        raise ValueError('Repeated/ambiguous command phrase: ' + phrase)
                    self.phrases[key] = intent
                    self.grammar.append(phrase)

    def intent(self, text):
        # Vosk unknown tokens must not disappear through punctuation normalization.
        if not isinstance(text, str) or len(text) > 256 or '[' in text or '<' in text:
            return None
        return self.phrases.get(normalize(text))
