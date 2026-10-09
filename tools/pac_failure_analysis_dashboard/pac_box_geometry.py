#!/usr/bin/env python3
"""Conservatively correlate new V4.4/V4.6 spawned SDF box sizes with a live run.

Only files created after observer startup are trusted: reused box_01 IDs in stale
log folders must NOT determine a box's visual dimensions in today's Gazebo run.
If there is no verified source file, the browser renders a position marker only.
"""
from __future__ import annotations
import math
from pathlib import Path
import re
import time
import xml.etree.ElementTree as ET

BOX_RE=re.compile(r'^box_\d{2,3}\.sdf$')
MAX_SDF_BYTES=2*1024*1024


def parse_spawned_box_sdf(data):
    if len(data)>MAX_SDF_BYTES:raise ValueError('oversized SDF')
    root=ET.fromstring(data)
    if root.tag!='sdf':raise ValueError('not SDF')
    for loc in ('.//model/link/visual/geometry/box/size',
                './/model/link/collision/geometry/box/size'):
        raw=root.findtext(loc)
        if not raw:continue
        try:values=[float(x) for x in raw.split()]
        except ValueError:continue
        if len(values)==3 and all(math.isfinite(v) and 0<v<4 for v in values):return values
    raise ValueError('box visual geometry unavailable')


class SpawnedBoxSizes:
    """Append-only verified names from SDF files newer than observer startup."""
    def __init__(self,repo,start_epoch=None,manifest=None):
        self.root=Path(repo).expanduser().absolute()/'logs'/'v44_generator_cycle'
        self.start_epoch=time.time() if start_epoch is None else float(start_epoch)
        self.known={}
        self.repo=Path(repo).expanduser().absolute()
        self.manifest=manifest
        self.bound_identity=None
        self.allowed_names=set()

    def refresh(self):
        from pac_integration import load_binding, load_catalog, run_identity, BindingError
        try:binding=load_binding(self.repo,self.manifest)
        except (BindingError,OSError):binding=None
        if binding is not None:
            identity=run_identity(binding)
            if identity!=self.bound_identity:
                self.known={};self.allowed_names=set();self.bound_identity=identity
            catalog=load_catalog(binding,self.repo)
            self.allowed_names=set(catalog)
            for name,size in catalog.items():
                self.known[name]=((binding['run_id'],name),size,'run_catalog')
            return
        if self.bound_identity is not None:
            self.known={};self.allowed_names=set();self.bound_identity=None
        if not self.root.is_dir():return
        try:runs=sorted((p for p in self.root.iterdir() if p.is_dir()),
                        key=lambda p:p.stat().st_mtime,reverse=True)[:3]
        except OSError:return
        checked=set()
        for run in runs:
            folder=run/'boxes'
            if not folder.is_dir():continue
            try:entries=list(folder.iterdir())[:100]
            except OSError:continue
            for file in entries:
                if not BOX_RE.fullmatch(file.name) or file.is_symlink() or not file.is_file() or file.stem in checked:continue
                try:
                    st=file.stat()
                    if st.st_mtime<self.start_epoch-1 or st.st_size>MAX_SDF_BYTES:continue
                    checked.add(file.stem)
                    identity=(str(run),file.stem)
                    if file.stem in self.known and self.known[file.stem][0]==identity:continue
                    with file.open('rb') as fp:dimensions=parse_spawned_box_sdf(fp.read(MAX_SDF_BYTES+1))
                    self.known[file.stem]=(identity,dimensions)
                except (ValueError,OSError,ET.ParseError):continue

    def apply(self,poses):
        for p in poses:
            record=self.known.get(p['name'])
            if record and (re.fullmatch(r'box_\d{2,3}',p['name']) or p['name'] in self.allowed_names):
                p['size']=record[1][:]
                p['size_source']=record[2] if len(record)>2 else 'spawn_sdf'
        return poses
