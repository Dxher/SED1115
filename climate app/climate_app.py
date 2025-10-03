import pandas as pd
import numpy as np
import plotly.express as px
import streamlit as st


# ------------------------
# Functions
# ------------------------

def _val(value):
    # Format a numeric value or return '—' if missing.
    return (f"{value:.1f}" if pd.notna(value) else "—")

def _get(row, name):
    # Safely get a value from a Series/row.
    return row.get(name, np.nan) if row is not None else np.nan

# show B as main value and green little comparisson (delta)
def _metric_B_vs_A(label: str, key: str):
    a = _get(day_data,   key)  # day A
    b = _get(day_data_b, key)  # day B
    value_str = _val(b)  # show B as the main number
    if pd.notna(a) and pd.notna(b):
        delta_str = f"{(b - a):+.1f}"   # signed delta
    else:
        delta_str = "—"
    st.metric(label, value=value_str, delta=delta_str)

# Remove unnamed columns
def _clean_series(s: pd.Series):
    return s.drop(labels=[col for col in s.index if "Unnamed:" in col], errors="ignore")


# ------------------------
# Setup dataframe
# ------------------------

# Load the CSV file into a dataframe
file_path = 'en_climate_daily_ON_6105976_2000_P1D.csv'
df = pd.read_csv(file_path)

# Drop all columns ending with "Flag"
flag_cols = [col for col in df.columns if col.endswith("Flag")]

# Drop specific unwanted columns
cols_to_drop = [
    "Climate ID", "Data Quality", "Longitude (x)",
    "Latitude (y)", "Heat Deg Days (°C)", "Cool Deg Days (°C)"
    ]

# Shrunken DF
df = df.drop(columns=cols_to_drop + flag_cols, errors="ignore")

# Ensure columns are numeric (prevents numbers as strings (e.g., "3.4", "—", "M"))
for col in df.columns:
    if col not in ["Station Name", "Date/Time"] and df[col].dtype == "object": #Often object is strings
        df[col] = pd.to_numeric(df[col], errors="coerce") #Make errors NaN (typically letters or empty spaces)

# Convenience: unique station & year
station_name = str(df["Station Name"][0])
year_value   = int(df["Year"][0])



# ------------------------
# UI
# ------------------------

st.set_page_config(page_title="Climate Viewer", layout="centered") #or wide

# Header 
st.markdown(
    f"<h2 style='text-align:center;margin-top:0'>{station_name}'s climate in the year {year_value}</h2>",
    unsafe_allow_html=True, #Streamlit option to render HTML instead of reading the text as text 
)

# Sub header
st.markdown(
    f"<h3 style='text-align:left;margin-top:0'>Select a day:</h3>",
    unsafe_allow_html=True, 
)

# Month & Day drop down boxes
col_month, col_day = st.columns([1, 1]) # streamlit column containers

months_available = sorted(df["Month"].dropna().unique().astype(int).tolist()) #sort list, remove missing values (NaN), keep one of each month, ensure ints, convert array to list.
with col_month:
    month = st.selectbox("Month", months_available, format_func=lambda m: f"{m:02d}")

days_available = sorted(df[df["Month"]== month]["Day"].dropna().unique().astype(int).tolist())
with col_day:
    if days_available:
        day = st.selectbox("Day", days_available, format_func=lambda d: f"{d:02d}")
    else:
        day = None
        st.warning("No days available for this month in the dataset.")

# Category selector
labels = ["🌡️ Temperature", "🌧️ Precipitation", "❄️ Snow", "💨 Wind"]
picked = st.radio("Data category", labels, horizontal=True)
category = picked.split(" ", 1)[1]   # Splits the string once on the first space. index 1 ([1]) is the category


# Column groupings per category
groups = {
    "Temperature": {
        "columns": [col for col in ["Max Temp (°C)", "Min Temp (°C)", "Mean Temp (°C)"]], #make a column for each list item
        "y_title": "Temperature (°C)"
    },
    "Precipitation": {
        "columns": [col for col in ["Total Rain (mm)", "Total Precip (mm)"]],
        "y_title": "Precipitation (mm)"
    },
    "Snow": {
        "columns": [col for col in ["Total Snow (cm)", "Snow on Grnd (cm)"]],
        "y_title": "Snow (cm)"
    },
    "Wind": {
        "columns": [col for col in ["Spd of Max Gust (km/h)"]],
        "y_title": "Wind (km/h)"
    },
}

y_cols = groups[category]["columns"]
y_axis_title = groups[category]["y_title"]



# ------------------------
# Extract a specific day's data
# ------------------------

# Data Row for the selected day
day_data = None
if day is not None:
    # The day row. If both month and day match, it will index to that day (boolean indexing). We use & because it's a pandas series
    row = df[(df["Month"] == month) & (df["Day"] == day)]
    #returns a Series (one value per column). 
    day_data = row.iloc[0] if not row.empty else None 


# Selected-day metrics by category
if category == "Temperature":
    col_a, col_b, col_c = st.columns(3)

    #check def for _get
    max_t  = _get(day_data, "Max Temp (°C)")
    min_t  = _get(day_data, "Min Temp (°C)")
    mean_t = _get(day_data, "Mean Temp (°C)")

    #check def for _val
    col_a.metric("Max Temp (°C)",  _val(max_t))
    col_b.metric("Min Temp (°C)",  _val(min_t))
    col_c.metric("Mean Temp (°C)", _val(mean_t))

elif category == "Precipitation":
    col_a, col_b = st.columns(2)

    rain   = _get(day_data, "Total Rain (mm)")
    precip = _get(day_data, "Total Precip (mm)")

    col_a.metric("Total Rain (mm)",   _val(rain))
    col_b.metric("Total Precip (mm)", _val(precip))

elif category == "Snow":
    col_a, col_b = st.columns(2)

    snow_total = _get(day_data, "Total Snow (cm)")
    snow_grnd  = _get(day_data, "Snow on Grnd (cm)")

    col_a.metric("Total Snow (cm)",    _val(snow_total))
    col_b.metric("Snow on Grnd (cm)",  _val(snow_grnd))

elif category == "Wind":
    # Show speed; optionally direction if present
    has_dir = "Dir of Max Gust (10s deg)" in df.columns

    if has_dir:
        col_a, col_b = st.columns(2)
    else:
        col_a = st.columns(1)[0]

    gust_spd = _get(day_data, "Spd of Max Gust (km/h)")
    col_a.metric("Max Gust Speed (km/h)", _val(gust_spd))

    if has_dir:
        gust_dir = _get(day_data, "Dir of Max Gust (10s deg)")
        # Direction is categorical. display as an integer
        col_b.metric("Max Gust Dir (10s deg)", _val(gust_dir))


# ------------------------
# Chart for the selected month across days
# ------------------------

#filters months, copy's the list to potentially avoid chained-assignment warnings, and sorts days
month_df = df[df["Month"] == month].copy().sort_values("Day")

# If no columns for this category exist (Temp/Precip/Snow/Wind), inform the user
if not y_cols:
    st.error(f"No columns found for {category} in this dataset.")

else:
    # Selects only the X axis (Day) plus the Y series you want. Melt compresses the columns into Day, Series and Value instead of having multiple columns. Good for plotly
    plot_df = month_df[["Day"] + y_cols].melt(id_vars="Day", value_vars=y_cols,
                                              var_name="Series", value_name="Value")

    # Plotly line chart
    fig = px.line(
        plot_df,
        x="Day",
        y="Value",
        color="Series",
        markers=True,
        title=f"{category} for {station_name} in {year_value}-{month:02d}",
    )
    fig.update_layout(
        xaxis_title="Day of Month",
        yaxis_title=y_axis_title,
        legend_title="",
        margin=dict(l=10, r=10, t=40, b=10),
    )

    # Highlight the selected day (vertical ruler)
    if day is not None and len(month_df):
        fig.add_vline(x=day, line_dash="dash", annotation_text=f"Day {day}", annotation_position="top")

    #Display chart
    st.plotly_chart(fig, use_container_width=True)



# ------------------------
# show the selected-day record for export
# ------------------------
with st.expander("Selected day - full record"):
    if day_data is not None:
        show = day_data.drop(labels=[col for col in day_data.index if "Unnamed:" in col], errors="ignore")
        st.dataframe(show.to_frame(name="Value"))
    else:
        st.write("No record found for this date.")



# ------------------------
# Compare two days
# ------------------------
st.markdown("### Compare two days")
enable_compare = st.toggle("Enable comparison", value=False)

day_data_b, month_b, day_b = None, None, None
if enable_compare:
    col_m2, col_d2 = st.columns([1, 1])
    with col_m2:
        # default Month (B) to Month (A) for convenience
        default_idx = months_available.index(month)
        month_b = st.selectbox("Month (B)", months_available, index=default_idx, format_func=lambda m: f"{m:02d}")

    days_available_b = sorted(df.loc[df["Month"] == month_b, "Day"].dropna().unique().astype(int).tolist())
    with col_d2:
        if days_available_b:
            day_b = st.selectbox("Day (B)", days_available_b, format_func=lambda d: f"{d:02d}")
            row_b = df[(df["Month"] == month_b) & (df["Day"] == day_b)]
            day_data_b = row_b.iloc[0] if not row_b.empty else None
        else:
            st.warning("No days available for month (B).")

    # If B isn't chosen yet, prompt; otherwise show B vs A metrics
    if day_data_b is None:
        st.info("Pick Month (B) and Day (B) to compare.")
    else:
        if category == "Temperature":
            c1, c2, c3 = st.columns(3)
            with c1: _metric_B_vs_A("Max Temp (°C)",  "Max Temp (°C)")
            with c2: _metric_B_vs_A("Min Temp (°C)",  "Min Temp (°C)")
            with c3: _metric_B_vs_A("Mean Temp (°C)", "Mean Temp (°C)")

        elif category == "Precipitation":
            c1, c2 = st.columns(2)
            with c1: _metric_B_vs_A("Total Rain (mm)",   "Total Rain (mm)")
            with c2: _metric_B_vs_A("Total Precip (mm)", "Total Precip (mm)")

        elif category == "Snow":
            c1, c2 = st.columns(2)
            with c1: _metric_B_vs_A("Total Snow (cm)",   "Total Snow (cm)")
            with c2: _metric_B_vs_A("Snow on Grnd (cm)", "Snow on Grnd (cm)")

        elif category == "Wind":
            cols = st.columns(2)
            with cols[0]: _metric_B_vs_A("Max Gust Speed (km/h)", "Spd of Max Gust (km/h)")
            if "Dir of Max Gust (10s deg)" in df.columns:
                with cols[1]: _metric_B_vs_A("Max Gust Dir (10s deg)", "Dir of Max Gust (10s deg)", decimals=0)


    # ------------------------
    # show comparisson record for export
    # ------------------------
    with st.expander("Comparison – full record"):
        if day_data is None or day_data_b is None:
            st.write("Pick both Day (A) and Day (B) to see a side-by-side comparison.")
        else:
            df = pd.DataFrame({
                "A": _clean_series(day_data),
                "B": _clean_series(day_data_b),
            })
            df["Difference (B − A)"] = (
                pd.to_numeric(df["B"], errors="coerce")
                - pd.to_numeric(df["A"], errors="coerce")
            ).round(2)
            df = df.reset_index().rename(columns={"index": "No."})
            st.dataframe(df, use_container_width=True, hide_index=True)          
