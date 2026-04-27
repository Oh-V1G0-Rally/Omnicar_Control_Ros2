from __future__ import annotations

from pathlib import Path

import pandas as pd

from common import SUMMARY_HEADERS


def _load_existing_summary(summary_csv_path: Path) -> pd.DataFrame:
    if not summary_csv_path.exists():
        return pd.DataFrame(columns=SUMMARY_HEADERS)

    current_df = pd.read_csv(summary_csv_path)
    if "test_id" in current_df.columns:
        return current_df

    if "parameter" not in current_df.columns:
        return pd.DataFrame(columns=SUMMARY_HEADERS)

    current_df = current_df.drop_duplicates(subset="parameter", keep="last")
    transposed = current_df.set_index("parameter").transpose().reset_index()
    if "test_id" in transposed.columns:
        transposed = transposed.drop(columns=["index"])
    else:
        transposed = transposed.rename(columns={"index": "test_id"})
    for header in SUMMARY_HEADERS:
        if header not in transposed.columns:
            transposed[header] = ""
    return transposed[SUMMARY_HEADERS]


def _write_transposed_summary(summary_csv_path: Path, combined: pd.DataFrame) -> None:
    rows = []
    for header in SUMMARY_HEADERS:
        output_row = {"parameter": header}
        for _, test_row in combined.iterrows():
            output_row[str(test_row["test_id"])] = test_row.get(header, "")
        rows.append(output_row)

    pd.DataFrame(rows).to_csv(summary_csv_path, index=False)


def update_global_comparison_csv(summary_csv_path: Path, row: dict) -> None:
    summary_csv_path.parent.mkdir(parents=True, exist_ok=True)

    new_df = pd.DataFrame([{key: row.get(key, "") for key in SUMMARY_HEADERS}])
    current_df = _load_existing_summary(summary_csv_path)
    current_df = current_df[current_df["test_id"] != row["test_id"]]
    combined = new_df if current_df.empty else pd.concat([current_df, new_df], ignore_index=True)

    combined = combined[SUMMARY_HEADERS]
    combined = combined.sort_values(by=["date", "test_id"]).reset_index(drop=True)
    _write_transposed_summary(summary_csv_path, combined)
