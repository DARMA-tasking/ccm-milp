#                           DARMA Toolkit v. 1.5.0
#
# Copyright 2019-2024 National Technology & Engineering Solutions of Sandia, LLC
# (NTESS). Under the terms of Contract DE-NA0003525 with NTESS, the U.S.
# Government retains certain rights in this software.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Questions? Contact darma@sandia.gov
#

import math
from dataclasses import dataclass, fields
import yaml

@dataclass
class Configuration:
    """CCM model parameters"""
    ranks_per_node: int = 1
    is_fwmp: bool = False
    beta: float = 0.0
    gamma: float = 0.0
    delta: float = 0.0
    rank_memory_bound: float = math.inf
    node_memory_bound: float = math.inf
    preserve_clusters: bool = False

    def __post_init__(self):
        if self.ranks_per_node < 1:
            raise ValueError(f"ranks_per_node must be at least 1, got {self.ranks_per_node}")
        for name in ("beta", "gamma", "delta"):
            if getattr(self, name) < 0:
                raise ValueError(f"{name} must be non-negative, got {getattr(self, name)}")
        for name in ("rank_memory_bound", "node_memory_bound"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive, got {getattr(self, name)}")

    @property
    def is_comcp(self) -> bool:
        """Whether this is the compute-only memory-constrained problem"""
        return not self.is_fwmp

    @classmethod
    def from_dict(cls, params: dict) -> "Configuration":
        """Build a configuration from a dict keyed by field name, coercing and validating values"""
        types = {f.name: f.type for f in fields(cls)}
        unknown = sorted(set(params) - set(types))
        if unknown:
            raise ValueError(
                f"unknown parameter(s): {', '.join(unknown)}; valid parameters are: {', '.join(types)}")
        return cls(**{k: _coerce(k, v, types[k]) for k, v in params.items()})

    @classmethod
    def from_yaml(cls, path: str) -> "Configuration":
        """Build a configuration from a YAML file of parameters"""
        return cls.from_dict(load_yaml(path))

    def summary(self) -> str:
        """Human-readable description of the configured problem"""
        model = "Full Work Model" if self.is_fwmp else "Compute-Only Memory-Constrained"
        lines = [
            f"# Configured {model} problem with:",
            f"  {self.ranks_per_node} rank{'s' if self.ranks_per_node > 1 else ''} per node",
            f"  beta = {self.beta}",
            f"  gamma = {self.gamma}",
            f"  delta = {self.delta}"]
        if self.rank_memory_bound < math.inf:
            lines.append(f"  rank memory bound = {self.rank_memory_bound}")
        if self.node_memory_bound < math.inf:
            lines.append(f"  node memory bound = {self.node_memory_bound}")
        if self.preserve_clusters:
            lines.append("  while preserving block clusters")
        return "\n".join(lines)

def load_yaml(path: str) -> dict:
    """Load a YAML file whose top level is a mapping"""
    with open(path, encoding="utf-8") as f:
        params = yaml.safe_load(f)
    if params is None:
        return {}
    if not isinstance(params, dict):
        raise ValueError(f"{path}: expected a mapping of parameter names to values")
    return params

def _coerce(name: str, value, target: type):
    """Convert a parameter value to its field type"""
    if target is bool:
        if not isinstance(value, bool):
            raise ValueError(f"{name} must be true or false, got {value!r}")
        return value
    expected = f"{name} must be {'an integer' if target is int else 'a number'}, got {value!r}"
    try:
        converted = target(value)
    except (TypeError, ValueError) as e:
        raise ValueError(expected) from e
    if isinstance(value, bool) or (target is int and converted != value):
        raise ValueError(expected)
    return converted
