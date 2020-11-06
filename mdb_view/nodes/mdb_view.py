#!/usr/bin/env python3

from view import app
from view import view

view.run('/home/gabriel/Documents/MDB_VIEW/src/mdb_common/config/ltm_2020_one_box.yaml')

def setup_node():
    app.run_server(debug=True)

if __name__ == '__main__':
    setup_node()
