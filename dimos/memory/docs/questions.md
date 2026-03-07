# Questions

1. "where was I when this log line was added?"
- pose lookup, corelating to log lines found
- assume log line has a pose associated
- assume there are multiple log lines matching a search

2. "how long have I been observing the red socks currently in view?"
- how many times did I see them before?
- temporal duration tracking + observation frequency

3. "how many people did I see during last week?"
- assume we are generating a facial recognition db — is this matching a face detection stream, then embeddings? then we are searching over that stream?

4. "where did you see red socks during last week?"
- we query for red socks embedding similarity, then feed this data into a VLM that further filters for socks
- is this data output into some table? is it like an ObservationSet again?
- then we can create a map (costmap) of red socks?

5. "did anyone ever open this door? at what times did I see this door open? who opened it?"
- event detection + temporal querying of state changes

6. "I have a transcription log (STT) and voice embeddings, how do I figure out who is saying what?"
- cross-stream correlation: audio → identity

7. "I have parallel voice and facial recognition streams, how do I correlate voice to people?"
- I don't see all people speaking at all times
- multi-modal fusion with incomplete overlap

8. "what's different in this room compared to yesterday?"
- comparing scene snapshots across time, diffing object sets
- requires baseline modeling / temporal comparison

9. "show me everywhere the cat went today"
- continuous spatial tracking over time, not point queries
- dense pose-stream retrieval + path aggregation

10. "what happened in the 30 seconds before the vase fell?"
- event-anchored temporal window across all streams
- multi-stream temporal slicing relative to a detected event

11. "when was the last time I did NOT see the cat in the apartment?"
- negation query — finding gaps in an observation stream
- architecturally different from presence queries

12. "what time does the mailman usually come?"
- aggregation across days, extracting temporal regularity from sparse events
- cross-session pattern extraction

13. "what did robot-2 observe in the warehouse that I missed?"
- cross-agent memory diff
- session/robot-scoped queries and set difference across streams

14. "how far did I travel while carrying an object?"
- filtered pose integration — only accumulate distance when a parallel detection stream has a positive signal
- cross-stream conditional joins
