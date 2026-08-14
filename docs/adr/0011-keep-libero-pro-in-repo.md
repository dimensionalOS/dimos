# Keep LIBERO-PRO integration in the DimOS repository

The first LIBERO-PRO integration is in-repo DimOS work: its Evaluation, `LiberoConnection`, blueprint composition, benchmark manifests, tests, and container definition live in the main repository and use the existing built-in evaluation resolution path. We will not create a separate Python distribution, installation workflow, or external `dimos.evaluations` entry point for this change. Outside registration remains the registry's concern and is not an abstraction the LIBERO vertical slice needs to introduce.
