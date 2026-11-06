"""Module 4: LDA and honest evaluation. REFERENCE SOLUTION.

Read one function at a time (the stuck rule), close it, then type it yourself.
"""
import numpy as np

import simdata

TRAIN_DAYS = np.array(simdata.TRAIN_DAYS)
TEST_DAYS = np.array(simdata.TEST_DAYS)


def lda_fit(X, y, shrinkage=0.0):
    """Fit LDA: Gaussian classes that share ONE covariance matrix.

    Steps (Worksheet 4.1-4.2):
      1. z-score every feature with the training mean and SD (like StandardScaler)
      2. class means mu_k, and the pooled within-class covariance
             S = sum over windows of (z - mu_class)(z - mu_class)^T / (n - n_classes)
      3. shrink: S = (1 - shrinkage) S + shrinkage (trace(S)/p) I
      4. the linear score  delta_k(z) = z^T W[:, k] + b[k]  with
             W = S^-1 [mu_1 ... mu_K]   and   b_k = -1/2 mu_k^T S^-1 mu_k + ln(prior_k)
    Returns a dict with keys classes, mean, sd, W, b.
    """
    # TODO (about 12 lines). np.linalg.solve(S, M) computes S^-1 M without forming S^-1.
    # --- solution ---
    classes = np.unique(y)
    mean, sd = X.mean(axis=0), X.std(axis=0)
    sd = np.where(sd > 0, sd, 1.0)                     # a constant feature: leave it at 0
    Z = (X - mean) / sd
    mu = np.array([Z[y == k].mean(axis=0) for k in classes])          # (K, p)
    R = Z - mu[np.searchsorted(classes, y)]
    S = R.T @ R / (len(Z) - len(classes))
    p = S.shape[0]
    S = (1 - shrinkage) * S + shrinkage * np.trace(S) / p * np.eye(p)
    W = np.linalg.solve(S, mu.T)                                       # (p, K)
    prior = np.array([np.mean(y == k) for k in classes])
    b = -0.5 * np.sum(mu.T * W, axis=0) + np.log(prior)
    return {"classes": classes, "mean": mean, "sd": sd, "W": W, "b": b}
    # --- end ---


def lda_predict(model, X):
    """The class with the highest score delta_k for each row of X."""
    # TODO (2 lines): z-score X with the TRAINING mean and SD, then argmax over Z @ W + b
    # --- solution ---
    Z = (X - model["mean"]) / model["sd"]
    return model["classes"][np.argmax(Z @ model["W"] + model["b"], axis=1)]
    # --- end ---


def balanced_accuracy(y, yhat):
    """Mean over classes of the fraction of that class's windows predicted correctly (recall).
    Chance is 1/(number of classes) whatever the class sizes.   (Worksheet 4.3)"""
    # TODO (1 line)
    # --- solution ---
    return float(np.mean([np.mean(yhat[y == k] == k) for k in np.unique(y)]))
    # --- end ---


def shares_a_sample(starts, is_test, win=400):
    """For ONE recording: window starts, and which windows are test windows. Return, for each test
    window, whether some TRAINING window of the same recording overlaps it (shares a sample).
    Two windows overlap when their starts are less than `win` apart.   (Worksheet 4.4)"""
    # TODO (3 lines)
    # --- solution ---
    train = starts[~is_test]
    return np.array([np.any(np.abs(train - s) < win) for s in starts[is_test]], bool)
    # --- end ---


def cross_day(T, X=None, shrinkage=0.0):
    """Train once on every window of the training days, then score each test day on its own.
    Returns the list of per-day balanced accuracies, one per test day, in day order."""
    # TODO (5 lines): fit on T.day in TRAIN_DAYS, then loop over TEST_DAYS
    # --- solution ---
    X = T.X if X is None else X
    tr = np.isin(T.day, TRAIN_DAYS)
    model = lda_fit(X[tr], T.y[tr], shrinkage)
    return [balanced_accuracy(T.y[T.day == d], lda_predict(model, X[T.day == d])) for d in TEST_DAYS]
    # --- end ---


# ------------------------------------------------------------------ provided from here down
SPLIT, GAP_S = 0.6, 0.25        # within-day: first 60 % trains, last 40 % tests, after a 0.25 s gap


def early(T):
    return T.t1 <= SPLIT * T.dur


def late(T):
    return T.t0 >= SPLIT * T.dur + GAP_S


def random_split(T, seed=0, X=None, shrinkage=0.0):
    """Pool the training days and split WINDOWS at random, 75 / 25 (emgdec/protocols.py,
    random_split). Neighbouring windows share 75 % of their samples, so this leaks.
    Returns (balanced accuracy, fraction of test windows that share a sample with training)."""
    X = T.X if X is None else X
    idx = np.flatnonzero(np.isin(T.day, TRAIN_DAYS))
    perm = np.random.default_rng(seed).permutation(idx)
    te, tr = perm[: len(idx) // 4], perm[len(idx) // 4:]
    model = lda_fit(X[tr], T.y[tr], shrinkage)
    acc = balanced_accuracy(T.y[te], lda_predict(model, X[te]))
    is_test = np.zeros(len(T.y), bool)
    is_test[te] = True
    shared = np.concatenate([shares_a_sample(T.start[T.rec == r], is_test[T.rec == r])
                             for r in np.unique(T.rec[idx])])
    return acc, float(shared.mean())


def within_day(T, X=None, shrinkage=0.0, days=TEST_DAYS):
    """Calibrate on the day: train on the first 60 % of each recording, test on its last 40 %."""
    X = T.X if X is None else X
    accs = []
    for d in days:
        m = T.day == d
        tr, te = m & early(T), m & late(T)
        accs.append(balanced_accuracy(T.y[te], lda_predict(lda_fit(X[tr], T.y[tr], shrinkage), X[te])))
    return accs


def cross_day_parts(T, X=None, shrinkage=0.0):
    """cross_day, scored separately on the first 60 % and the last 40 % of each test recording,
    the windows within_day tests on (DEBUG_LOG #9)."""
    X = T.X if X is None else X
    tr = np.isin(T.day, TRAIN_DAYS)
    model = lda_fit(X[tr], T.y[tr], shrinkage)
    out = {"early 60 %": [], "late 40 %": []}
    for d in TEST_DAYS:
        for name, part in (("early 60 %", early(T)), ("late 40 %", late(T))):
            m = (T.day == d) & part
            out[name].append(balanced_accuracy(T.y[m], lda_predict(model, X[m])))
    return out


def ledoit_wolf(X, y):
    """Background, not part of the worksheet: the Ledoit-Wolf shrinkage the real decoder gets from
    scikit-learn's shrinkage="auto". alpha = min(1, b^2 / d^2) on the z-scored, class-centred data."""
    classes = np.unique(y)
    sd = X.std(axis=0)
    Z = (X - X.mean(axis=0)) / np.where(sd > 0, sd, 1.0)
    R = Z - np.array([Z[y == k].mean(axis=0) for k in classes])[np.searchsorted(classes, y)]
    n, p = R.shape
    S = R.T @ R / n
    mu = np.trace(S) / p
    d2 = np.sum((S - mu * np.eye(p)) ** 2)
    b2 = (np.sum(np.sum(R ** 2, axis=1) ** 2) / n - np.sum(S ** 2)) / n
    return float(min(1.0, b2 / d2)) if d2 > 0 else 1.0


def mean_sd(v):
    v = np.asarray(v, float)
    return f"{100 * v.mean():.1f} % (SD {100 * v.std():.1f})"
