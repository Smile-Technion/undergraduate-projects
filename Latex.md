<script type="text/javascript" async
src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.2/MathJax.js? 
config=TeX-MML-AM_CHTML"
</script>

<hr />

### Lecture

- [Linear regression-2D-Model](#LR_2D)
  - [2D-Model-Generalization](#LR_G)
- [Linear regression-nD-Model](#LR_nD)

<hr />

## Linear regression 

<a name='LR_2D'></a>
### 2D Basic Model:

<hr />

#### Notations:


In linear regression we assume the connection between X and Y is linear:

$\hat{Y}_i=\omega_1 x_i+\omega_0 ~ $ trying to fit $ \omega_1,\omega_0$ 

in order to achieve the lowest error.(we want $\hat{Y}_i$ to be as close as possible to $Y_i$ )

<p align="center">
	<img src="/C096411/image/less1/Capture1.PNG" align="middle">
</p>


In order to achieve the learning goal($\hat{Y}_i$ to be as close as possible to $Y_i$) we want to minimize the sum of all  error(green line) as much as possible. 

<hr />

#### Formulation:

The goal is to find $\omega_0,\omega_1$ that minimize the error.

##### Finding $\omega_0,\omega_1$ that minimize this equation:

<p align="center">
	<img src="/C096411/image/less1/Capture4.PNG" align="middle">
</p>

<a name='LR_G'></a>


<hr />

#### Generalization :


Our main goal is to do well on the real world not on our training set S. 
Let’s assume the real world model is not a linear model but it's possible to draw a line such that
$\sum$errors(green lines) would be 0. in this case we can imagine the error as a independent variable with a
variance $\sigma$ and mean=0:

$\hat{Y}-Y=\epsilon$

##### such that:

$E(\hat{Y}-Y)=E(\epsilon)=0 \to E(\hat{Y})=E(Y)$

$E(\hat{\omega_1})=E(\omega_1)=\omega_1$

$E(\hat{\omega_0})=E(\omega_0)=\omega_0$

##### Lets prove the equality for $\hat{\omega_0}$ and $\hat{\omega_1}$ holds:


<img src="/C096411/image/less3/Capture3.PNG" align="middle">



<a name='LR_nD'></a>

<hr />

### General case nD Model:


Until now we only dealt with 2-dimension case Y=w1x+b This section goal is to generalize to n Dimensions :


#### The equation is now:

$$
\arg min {(Y-\hat{Y})}^2={(Y-X_i\omega)}^2 
$$


#### Finding $\omega$ that minimize this equation:

$ \frac{d}{d\omega }=2 X^{T}(Y-X\omega)=0\to \omega =\frac{X^T Y}{X^T X} $

<a name='PR'></a>



<hr />
