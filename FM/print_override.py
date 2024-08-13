import builtins

original_print = print

def custom_print(*args, **kwargs):
    # ログファイルへの書き込みを行えるようにする
    with open('fm_print_log.txt', 'a') as f:
        f.write(' '.join(map(str, args)) + '\n')
    
    original_print(*args, **kwargs)

# print関数をカスタム関数に置き換える
def printoverride():
    builtins.print = custom_print
