"""LDA baseline and a small 1D CNN."""
import numpy as np
import torch
import torch.nn as nn
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.metrics import balanced_accuracy_score
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler


def lda():
    # Shrinkage keeps the covariance well-conditioned when a board zeros a channel.
    return make_pipeline(StandardScaler(),
                         LinearDiscriminantAnalysis(solver="lsqr", shrinkage="auto"))


def lda_score(Xtr, ytr, Xte, yte):
    m = lda().fit(Xtr, ytr)
    return balanced_accuracy_score(yte, m.predict(Xte)), m


class EMGNet(nn.Module):
    def __init__(self, n_ch=4, n_cls=11):
        super().__init__()
        block = lambda i, o, k: [nn.Conv1d(i, o, k, padding=k // 2), nn.BatchNorm1d(o), nn.ReLU()]
        self.net = nn.Sequential(
            *block(n_ch, 32, 9), nn.MaxPool1d(2),
            *block(32, 64, 7), nn.MaxPool1d(2),
            *block(64, 64, 5),
            nn.AdaptiveAvgPool1d(1), nn.Flatten(), nn.Dropout(0.3), nn.Linear(64, n_cls))

    def forward(self, x):          # x: (batch, channels, time)
        return self.net(x)


def _predict(model, X, bs=2048):
    model.eval()
    out = []
    with torch.no_grad():
        for i in range(0, len(X), bs):
            out.append(model(torch.from_numpy(X[i:i + bs])).argmax(1).numpy())
    return np.concatenate(out)


def train_cnn(Xtr, ytr, Xva, yva, n_cls, seed, epochs=15, bs=256, gain_aug=None, flip_aug=False,
              log=print):
    """Windows are (n, time, ch) float32, already scaled. Keeps the best-validation epoch.

    `gain_aug = g` multiplies every channel of every training window by an independent
    log-uniform factor in [1/g, g]: a stand-in for electrode and skin gain drifting
    between days. `flip_aug` inverts each channel of each window with probability 1/2:
    a reversed electrode lead, which happens several times in this dataset.
    """
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    Xtr, Xva = (np.ascontiguousarray(np.swapaxes(a, 1, 2)) for a in (Xtr, Xva))
    model = EMGNet(Xtr.shape[1], n_cls)
    opt = torch.optim.AdamW(model.parameters(), lr=1e-3, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, epochs)
    counts = np.bincount(ytr, minlength=n_cls)
    loss_fn = nn.CrossEntropyLoss(weight=torch.tensor(len(ytr) / (n_cls * np.maximum(counts, 1)),
                                                      dtype=torch.float32))
    best, best_state, history = -1.0, None, []
    for ep in range(epochs):
        model.train()
        order = rng.permutation(len(Xtr))
        for i in range(0, len(order), bs):
            idx = order[i:i + bs]
            xb = torch.from_numpy(Xtr[idx])
            if gain_aug:
                g = np.exp(rng.uniform(-np.log(gain_aug), np.log(gain_aug), (len(idx), xb.shape[1], 1)))
                xb = xb * torch.from_numpy(g.astype(np.float32))
            if flip_aug:
                s = rng.choice([-1.0, 1.0], (len(idx), xb.shape[1], 1))
                xb = xb * torch.from_numpy(s.astype(np.float32))
            yb = torch.from_numpy(ytr[idx]).long()
            opt.zero_grad()
            loss_fn(model(xb), yb).backward()
            opt.step()
        sched.step()
        va = balanced_accuracy_score(yva, _predict(model, Xva))
        history.append(va)
        log(f"    epoch {ep + 1:2d}  val bal-acc {va:.3f}")
        if va > best:
            best, best_state = va, {k: v.clone() for k, v in model.state_dict().items()}
    model.load_state_dict(best_state)
    return model, history


def cnn_predict(model, X):
    return _predict(model, np.ascontiguousarray(np.swapaxes(X, 1, 2)))
