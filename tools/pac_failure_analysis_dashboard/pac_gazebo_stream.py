"""Bounded incremental reader of `ign topic -e` Pose_V text (read only).

The ROS TFMessage bridge can erase Gazebo entity names. Never invent the names by
matching offsets or indices; read the native named Pose_V stream instead.
"""
from __future__ import annotations
import re
import time

MAX_TEXT_BYTES = 6 * 1024 * 1024


class PoseTextDecoder:
    """Split complete protobuf-text Pose_V frames from an ongoing topic echo.

    Supported framing: `---` delimiters, a new top-level `header {` after
    completed pose blocks, and idle flush after a complete message. Output
    messages are raw protobuf text for parse_ign_pose_text (names preserved).
    """
    def __init__(self):
        self.current=[]
        self.partial=''
        self.depth=0
        self.has_pose=False
        self.last_input=time.monotonic()
        self.errors=0

    def _take(self):
        if self.depth!=0 or not self.has_pose:
            return None
        value='\n'.join(self.current)+'\n'
        self.current=[]
        self.has_pose=False
        if len(value.encode('utf-8'))>MAX_TEXT_BYTES:
            self.errors+=1
            return None
        return value

    def feed(self, data):
        if not isinstance(data,str):
            raise TypeError('stream chunks must be decoded strings')
        self.last_input=time.monotonic()
        self.partial+=data
        ready=[]
        if len(self.partial)+sum(map(len,self.current))>MAX_TEXT_BYTES:
            self.current=[];self.partial='';self.depth=0;self.has_pose=False
            self.errors+=1
            return ready
        while '\n' in self.partial:
            line,self.partial=self.partial.split('\n',1)
            stripped=line.strip()
            if self.depth==0 and stripped in ('---','----','-----'):
                got=self._take()
                if got:ready.append(got)
                continue
            if self.depth==0 and stripped=='header {' and self.has_pose:
                got=self._take()
                if got:ready.append(got)
            if self.depth==0 and stripped=='pose {':
                self.has_pose=True
            self.current.append(line)
            self.depth+=line.count('{')-line.count('}')
            if self.depth<0:
                self.depth=0;self.current=[];self.has_pose=False;self.errors+=1
        return ready

    def flush_idle(self,idle_s=0.3):
        # Do not accept an incomplete protobuf frame as a valid observation.
        if time.monotonic()-self.last_input < idle_s or self.partial.strip():
            return None
        return self._take()


def usable_pose_topic(topics, requested=''):
    """Prefer continuously-updating dynamic Pose_V when available; do not guess worlds."""
    if requested:return requested
    candidates=[s.strip() for s in topics if re.fullmatch(
        r'/world/[^/]+/(?:dynamic_)?pose/info',s.strip())]
    dynamic=[s for s in candidates if s.endswith('/dynamic_pose/info')]
    return dynamic[0] if len(dynamic)==1 else (candidates[0] if len(candidates)==1 else '')


def choose_runtime_pose_topic(topics, *, world='', requested=''):
    """Fail closed on ambiguous worlds. Prefer dynamic pose for a selected world.

    Explicit topic is accepted only when currently advertised; no guessing an
    inactive world or silently switching to a different run.
    """
    matches=sorted({s.strip() for s in topics if re.fullmatch(
        r'/world/[^/]+/(?:dynamic_)?pose/info',s.strip())})
    if world:
        matches=[s for s in matches if s.startswith('/world/'+world+'/')]
    if requested:
        return requested if requested in matches else ''
    dynamic=[s for s in matches if s.endswith('/dynamic_pose/info')]
    if len(dynamic)==1:return dynamic[0]
    if len(matches)==1:return matches[0]
    return ''
