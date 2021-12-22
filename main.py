# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

#Import python packages
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.svm import SVR
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.metrics import accuracy_score
import pickle

if __name__ == '__main__':

    df = pd.read_csv(r"merge.csv",index_col=False)
    df.reset_index(drop=True, inplace=True)
    df.head()
    for x in df:
        if df[x].dtypes == "int64":
            df[x] = df[x].astype(float)
            print(df[x].dtypes)

    df = df.select_dtypes(exclude=['object'])
    df = df.fillna(df.mean())
    X = df.drop('class', axis=1)
    y = df['class']

    # X = df.values[:,0:2]
    # Y = df.values[:,2]
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=10)

    model = SVC()
    model.fit(X_train,y_train)
    predict = model.predict(X_test)
    print('accurscy is' ,accuracy_score(y_test,predict))
    pickle.dump(model, open("model.pkl", "wb"),protocol=2)

    loaded_model = pickle.load(open("model.pkl", 'rb'))
    predict = loaded_model.predict(np.array([1, 1]).reshape(1, -1))[0]
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
