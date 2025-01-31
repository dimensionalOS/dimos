# TODO: REMOVE - NOT USED

# from abc import ABC, abstractmethod
# import logging
# from dimos.exceptions.agent_token_strategy_exceptions import UnknownTokenStrategyError

# # TODO: Rename to AbstractTokenStrategy and move out impls to separate files

# # Priorities are:
# # * A tokenstartegy that doesnt care about output length, 
# # * A tokenstartegy that truncates, the end, middle, or beginning of the query.
# # * A tokenstartegy that fails if the query is too long.
# # * A tokenstartegy that silently fails if the query is too long.
# # * A tokenstartegy that results in a query that is too short.

# class AbstractTokenStrategy(ABC):
#     def __init__(self, **kwargs):
#         pass

#     @abstractmethod
#     def process(self, queries):
#         pass

# class TruncateTokenStrategy(AbstractTokenStrategy):
#     def __init__(self, max_tokens=16384, truncate_strategy='head', **kwargs):
#         """
#         Initialize the token truncation strategy.
#         Args:
#             max_tokens (int): Maximum number of tokens allowed.
#             truncate_strategy (str): Strategy for truncation ('head', 'tail', 'middle').
#         Raises:
#             ValueError: If an unrecognized truncation strategy is specified.
#         """
#         self.logger = logging.getLogger(self.__class__.__name__)
#         self.logger.info('Initializing TruncateTokenStrategy with max_tokens=%d and strategy=%s', max_tokens, truncate_strategy)

#         self.max_tokens = max_tokens
#         self.truncate_strategy = truncate_strategy.lower()

#         if self.truncate_strategy not in ['head', 'tail', 'middle']:
#             raise UnknownTokenStrategyError(f"Invalid truncate_strategy {truncate_strategy}. Expected 'head', 'tail', or 'middle'.")

#     def _truncate(self, text):
#         """
#         Truncate the input text according to the strategy and max_tokens.
#         Args:
#             text (str): Input text to truncate.
#         Returns:
#             str: Truncated text.
#         """
#         tokens = text.split()  # Tokenize by whitespace. Replace with a tokenizer if needed.
#         if len(tokens) <= self.max_tokens:
#             return text  # No truncation needed.

#         self.logger.info("Truncating text with strategy: %s", self.truncate_strategy)
#         if self.truncate_strategy == 'head':
#             truncated_tokens = tokens[-self.max_tokens:]  # Keep the last tokens.
#         elif self.truncate_strategy == 'tail':
#             truncated_tokens = tokens[:self.max_tokens]  # Keep the first tokens.
#         elif self.truncate_strategy == 'middle':
#             half = self.max_tokens // 2
#             truncated_tokens = tokens[:half] + tokens[-half:]  # Keep start and end.

#         return ' '.join(truncated_tokens)

#     def process(self, querey):
#         """
#         Processing a set of RAG queries, truncating each query as needed.
#         Args:
#             queries (list of str): List of queries to process.
#         Returns:
#             list of str: Truncated queries.
#         """
#         self.logger.info(f"Processing query: {querey}")

#         truncated_queries = [self.truncate(query) for query in querey]
#         return truncated_queries
