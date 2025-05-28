from pydantic import BaseModel, RootModel
from typing import List, Optional, Dict, Any

class BaseTopic(BaseModel):
    name: str
    type: str
    qos: Optional[int] = 10

class PublisherTopic(BaseTopic):
    freq: Optional[float] = 1.0

class SubscriberTopic(BaseTopic):
    callback: str

class TopicGroup(BaseModel):
    publishers: Optional[List[PublisherTopic]] = None
    subscribers: Optional[List[SubscriberTopic]] = None

# class NodeConfig(BaseModel):
#     topics: Optional[TopicGroup] = None

# class BaseRosConfig(RootModel[Dict[str, NodeConfig]]):
#     pass

class BaseRosConfig(BaseModel):
    topics: Optional[TopicGroup] = None