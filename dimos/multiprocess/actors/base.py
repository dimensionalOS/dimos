def deploy_actor(dask_client, actor_class, *args, **kwargs):
    return dask_client.submit(
        actor_class,
        *args,
        **kwargs,
        actor=True,
    ).result()
