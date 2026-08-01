import { Streamdown, type UrlTransform } from "streamdown";

const localOnlyUrl: UrlTransform = (url, key, node) => {
  if (node.tagName === "a" && key === "href" && url.startsWith("#")) {
    return url;
  }
  return null;
};

export function Markdown({ children }: { children: string }) {
  return (
    <Streamdown
      className="markdown"
      controls={false}
      mode="static"
      parseIncompleteMarkdown={false}
      skipHtml
      urlTransform={localOnlyUrl}
    >
      {children}
    </Streamdown>
  );
}
