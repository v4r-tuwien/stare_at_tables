# Stare at Tables
Action Server that records rosbags while the HSR moves around a plane extracted by the [table_extractor](https://rgit.acin.tuwien.ac.at/leitnermrks/table_extractor) staring at the center of the plane. 

## Usage
The action goal contains a single id that corresponds to a plane in the database. Make sure the mongodb database is running. The rosbag will be saved to the path defined as ```self.rosbag_path``` in the ```move_around_table.py``` script. 
The StareAtTablesAction looks like this: 
```
# Define the goal
int32 id 
---
# Define the result
---
# Define a feedback message
```
There is no result and no feedback. 