from tensorflow.examples.tutorials.mnist import input_data
mnist = input_data.read_data_sets("MNIST_data/", one_hot=True)

import tensorflow as tf
from tensorflow.python.saved_model import tag_constants
from matplotlib import pyplot as plt
from random import randint

model_path = "./model"
graph2 = tf.Graph()
with graph2.as_default():
    with tf.Session(graph=graph2) as sess:    

        tf.saved_model.loader.load(sess,
                                    [tag_constants.SERVING],
                                    model_path
                                    )
        pred2 = graph2.get_tensor_by_name("out:0")
        x = graph2.get_tensor_by_name('x:0')
        num = randint(0, mnist.test.images.shape[0])
        img = mnist.test.images[num]
        classification = sess.run(tf.argmax(pred2, 1), feed_dict={x: [img]})
        plt.imshow(img.reshape(28, 28), cmap=plt.cm.binary)
        plt.show()
        print 'NN predicted', classification[0]