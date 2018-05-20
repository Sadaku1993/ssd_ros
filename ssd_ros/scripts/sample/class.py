#coding:utf-8

class TestClass:
    def __init__(self, code, name):
        self.code = code
        self.name = name

classes = []
classes.append(TestClass(1, u'test1'))
classes.append(TestClass(2, u'test2'))
classes.append(TestClass(3, u'test3'))

# insertを利用するとindex値を指定して要素を追加可能
classes.insert(0, TestClass(0, u'test0'))

# 指定のindex値に存在する要素の削除を行い,削除された要素を戻り値として返す
classes.pop(3)

for cls in classes:
    print "===== Class ====="
    print 'code --> ' + str(cls.code)
    print 'name --> ' + cls.name
