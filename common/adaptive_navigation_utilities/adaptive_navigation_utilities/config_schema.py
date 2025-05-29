from pydantic import BaseModel, ConfigDict
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

class BaseRosConfig(BaseModel):
    model_config = ConfigDict(extra="allow") 
    extra: Optional[Dict[str, Any]] = None
    topics: Optional[TopicGroup] = None