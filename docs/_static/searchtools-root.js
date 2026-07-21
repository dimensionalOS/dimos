/* Keep Sphinx's native search, but make its directory-style result links root-relative. */
(function () {
  "use strict";

  const normalizeSearchLinks = () => {
    const contentRoot = document.documentElement.dataset.content_root || "./";
    document.querySelectorAll("#search-results a[href]").forEach((link) => {
      const href = link.getAttribute("href");
      if (!href || href.startsWith("/") || href.startsWith("#") || /^[a-z][a-z0-9+.-]*:/i.test(href)) {
        return;
      }
      const [path, fragment = ""] = href.split("#", 2);
      const canonicalPath = path.endsWith("/index/")
        ? path.slice(0, -6)
        : path === "index/"
          ? ""
          : path;
      const target = new URL(contentRoot + canonicalPath, document.baseURI);
      target.hash = fragment;
      link.href = target.href;
    });
  };

  normalizeSearchLinks();
  new MutationObserver(normalizeSearchLinks).observe(document.body, {
    childList: true,
    subtree: true,
  });
})();
