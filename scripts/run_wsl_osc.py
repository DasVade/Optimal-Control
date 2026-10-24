"""Headless OSC benchmark: run with the persistent WSL Python interpreter."""
import argparse
import importlib.metadata
import json
from pathlib import Path
import platform
import sys
from control_portfolio.osc_drake import run

parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--duration',type=float,default=.2);args=parser.parse_args()
args.output.mkdir(parents=True,exist_ok=False)
(args.output/'environment.json').write_text(json.dumps(dict(python=sys.version,executable=sys.executable,
    platform=platform.platform(),dependencies={name:importlib.metadata.version(name) for name in ['drake','numpy','scipy','osqp']},
    duration_s=args.duration),indent=2))
run(args.output,args.duration)
