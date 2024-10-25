from enum import Enum

class States(Enum):
    IDLE = 1
    COMMANDS_IN_QUEUE = 2
    EXECUTING_COMMANDS = 3
    EXECUTION_SUCCESS = 4
    EXECUTION_FAILED = 5
    STOPPING = 6
    ERROR = 7
    SHUTDOWN = 8


if __name__ == "__main__":
    # Access enum members
    print(States.IDLE)       # Output: States.RED
    print(States.IDLE.name)  # Output: 'IDLE'
    print(States.IDLE.value) # Output: 1

    # Iterating over enum members
    for state in States:
        print(state)
        
    print(States.IDLE == States.IDLE) # Output: True
    print(States.IDLE == States.COMMANDS_IN_QUEUE) # Output: False

