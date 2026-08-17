# Vendored Small Point-LIO

This directory is derived from
[`Yancey2023/small_point_lio`](https://github.com/Yancey2023/small_point_lio),
ROS 2 branch commit `b87754309d5a8603dfb8ec7631bf7665dc067b21`.

The upstream code is MIT licensed; see `LICENSE.txt`. Local integration changes
add an in-process frame callback and expose the accepted point-to-plane
information used by HybridSLAM covariance estimation. The original standalone
ROS component source is retained for provenance but is not compiled by
HybridSLAM.

The local integration also adds a conventional Velodyne PointCloud2 adapter
for M2DGR's `x/y/z/intensity/ring/time` layout. Frame point vectors are moved,
rather than copied, into HybridSLAM's bounded downstream worker; body-frame
clouds are generated lazily outside the Point-LIO estimator callback. Adapter
subscriptions accept caller-selected QoS and callback groups so live sensors
and reliable offline replay can use different transport policies.
