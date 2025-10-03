Anthony Daher 300233710

Test status: [![tests](https://github.com/Dxher/climate_app_SED1115/actions/workflows/tests.yml/badge.svg)](https://github.com/Dxher/climate_app_SED1115/actions/workflows/tests.yml)

Climate_app.py is a software that allows you to visualize climate data extracted from Environment Canada. The CSV must contain a year's worth of climate data (daily data) and not those displaying monthly data.

Climate_app.py allows you to compare two separate days within the year.

A Database is already provided within this repository. To execute the code:

1. Clone this repository onto your local device.
2. Open your command prompt or git bash
3. Install the necessary libraries if not owned, in other words, run this command on your terminal: pip install pandas numpy plotly streamlit
4. Once installed, navigate to the cloned repository (For example: cd Documents/SED1115/weatherapp)
5. Once in the cloned repo folder, run: streamlit run climate_app.py

Enjoy :)

6. If for whatever reason your streamlit didn't install in your path, this is a work around:
   cd <your-repo-folder>
   python -m pip install -U streamlit
   python -m streamlit run climate_app.py
