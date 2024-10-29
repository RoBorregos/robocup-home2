from abc import ABC, abstractmethod
from typing import List

class TasksBase(ABC):
    
    @abstractmethod
    def execute_command(self, command: str, complement: str) -> None:
        pass
    
    @abstractmethod
    def cancel_command(self, command: str) -> None:
        pass
    
    def get_tasks(self) -> List[str]:
        return [task.name for task in self.task_names]

    