#!/usr/bin/env python3
"""
Generate a perturbed MJCF model by scaling the mass and diagonal inertia
of all inertial elements.

For each <inertial> element, the same random scale factor is applied to
both mass and diaginertia:

    scale_i = 1 + delta_i
    delta_i ~ Uniform(-error_level, +error_level)

Example:
    python3 scripts/generate_perturbed_mjcf.py \
        --input models/mlivr_nominal.xml \
        --output models/generated/mlivr_inertial_err_10_seed00.xml \
        --error-level 0.10 \
        --seed 0
"""

import argparse
import copy
import random
import xml.etree.ElementTree as ET
from pathlib import Path


def parse_float_list(text: str) -> list[float]:
    """Parse a whitespace-separated list of floats."""
    return [float(x) for x in text.split()]


def format_float(value: float) -> str:
    """Format floats compactly while keeping sufficient precision."""
    return f"{value:.10g}"


def format_float_list(values: list[float]) -> str:
    """Format a list of floats for MJCF attributes."""
    return " ".join(format_float(v) for v in values)


def perturb_mjcf(
    input_path: Path,
    output_path: Path,
    error_level: float,
    seed: int,
    skip_base: bool = False,
) -> None:
    if not input_path.exists():
        raise FileNotFoundError(f"Input MJCF file not found: {input_path}")

    if error_level < 0.0:
        raise ValueError("--error-level must be non-negative.")

    rng = random.Random(seed)

    tree = ET.parse(input_path)
    root = tree.getroot()

    inertial_elements = root.findall(".//inertial")

    if not inertial_elements:
        raise RuntimeError("No <inertial> elements were found in the MJCF file.")

    log_lines = []
    log_lines.append(f"# input: {input_path}")
    log_lines.append(f"# output: {output_path}")
    log_lines.append(f"# error_level: ±{100.0 * error_level:.1f}%")
    log_lines.append(f"# seed: {seed}")
    log_lines.append("")
    log_lines.append("index,body_name,scale,old_mass,new_mass,old_diaginertia,new_diaginertia")

    perturbed_count = 0

    # Each <inertial> is usually inside a <body>.
    # ElementTree does not provide a parent pointer, so we recursively traverse bodies.
    for idx, body in enumerate(root.findall(".//body")):
        body_name = body.attrib.get("name", f"body_{idx}")
        inertial = body.find("inertial")

        if inertial is None:
            continue

        if skip_base and body_name == "base_link":
            continue

        if "mass" not in inertial.attrib:
            continue

        scale = 1.0 + rng.uniform(-error_level, error_level)

        old_mass = float(inertial.attrib["mass"])
        new_mass = old_mass * scale
        inertial.attrib["mass"] = format_float(new_mass)

        old_diaginertia_text = inertial.attrib.get("diaginertia", None)

        if old_diaginertia_text is not None:
            old_diaginertia = parse_float_list(old_diaginertia_text)

            if len(old_diaginertia) != 3:
                raise ValueError(
                    f"Invalid diaginertia for body '{body_name}': "
                    f"{old_diaginertia_text}"
                )

            new_diaginertia = [v * scale for v in old_diaginertia]

            # Safety check: MuJoCo requires positive inertia values.
            if any(v <= 0.0 for v in new_diaginertia):
                raise ValueError(
                    f"Non-positive inertia generated for body '{body_name}'."
                )

            inertial.attrib["diaginertia"] = format_float_list(new_diaginertia)
        else:
            old_diaginertia = []
            new_diaginertia = []

        log_lines.append(
            f"{perturbed_count},"
            f"{body_name},"
            f"{format_float(scale)},"
            f"{format_float(old_mass)},"
            f"{format_float(new_mass)},"
            f"\"{format_float_list(old_diaginertia)}\","
            f"\"{format_float_list(new_diaginertia)}\""
        )

        perturbed_count += 1

    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Preserve XML declaration.
    tree.write(output_path, encoding="utf-8", xml_declaration=True)

    log_path = output_path.with_suffix(output_path.suffix + ".log.csv")
    log_path.write_text("\n".join(log_lines) + "\n", encoding="utf-8")

    print(f"Generated perturbed MJCF: {output_path}")
    print(f"Generated perturbation log: {log_path}")
    print(f"Perturbed inertial elements: {perturbed_count}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate an MJCF file with random mass/inertia perturbations."
    )
    parser.add_argument(
        "--input",
        required=True,
        type=Path,
        help="Path to the nominal MJCF file.",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="Path to the output perturbed MJCF file.",
    )
    parser.add_argument(
        "--error-level",
        required=True,
        type=float,
        help="Relative error level. For example, 0.10 means ±10%.",
    )
    parser.add_argument(
        "--seed",
        required=True,
        type=int,
        help="Random seed for reproducibility.",
    )
    parser.add_argument(
        "--skip-base",
        action="store_true",
        help="Do not perturb the inertial parameters of body name='base_link'.",
    )

    args = parser.parse_args()

    perturb_mjcf(
        input_path=args.input,
        output_path=args.output,
        error_level=args.error_level,
        seed=args.seed,
        skip_base=args.skip_base,
    )


if __name__ == "__main__":
    main()
